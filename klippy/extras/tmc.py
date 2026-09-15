# Common helper code for TMC stepper drivers
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, collections
import stepper
from . import bulk_sensor


######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

class FieldHelper:
    def __init__(self, all_fields, signed_fields=[], field_formatters={},
                 registers=None):
        self.all_fields = all_fields
        self.signed_fields = {sf: 1 for sf in signed_fields}
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = collections.OrderedDict()
        self.field_to_register = { f: r for r, fields in self.all_fields.items()
                                   for f in fields }
    def lookup_register(self, field_name, default=None):
        return self.field_to_register.get(field_name, default)
    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        field_value = (reg_value & mask) >> ffs(mask)
        if field_name in self.signed_fields and ((reg_value & mask)<<1) > mask:
            field_value -= (1 << field_value.bit_length())
        return field_value
    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.registers[reg_name] = new_value
        return new_value
    def set_config_field(self, config, field_name, default):
        # Allow a field to be set from the config file
        config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        elif field_name in self.signed_fields:
            val = config.getint(config_name, default,
                                minval=-(maxval//2 + 1), maxval=maxval//2)
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        return self.set_field(field_name, val)
    def pretty_format(self, reg_name, reg_value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            field_value = self.get_field(field_name, reg_value, reg_name)
            sval = self.field_formatters.get(field_name, str)(field_value)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))
    def get_reg_fields(self, reg_name, reg_value):
        # Provide fields found in a register
        reg_fields = self.all_fields.get(reg_name, {})
        return {field_name: self.get_field(field_name, reg_value, reg_name)
                for field_name, mask in reg_fields.items()}


######################################################################
# Periodic error checking
######################################################################

class TMCErrorCheck:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        name_parts = config.get_name().split()
        self.stepper_name = ' '.join(name_parts[1:])
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.check_timer = None
        self.last_drv_status = self.last_drv_fields = None
        # Setup for GSTAT query
        reg_name = self.fields.lookup_register("drv_err")
        if reg_name is not None:
            self.gstat_reg_info = [0, reg_name, 0xffffffff, 0xffffffff, 0]
        else:
            self.gstat_reg_info = None
        self.clear_gstat = True
        # Setup for DRV_STATUS query
        self.irun_field = "irun"
        reg_name = "DRV_STATUS"
        mask = err_mask = cs_actual_mask = 0
        if name_parts[0] == 'tmc2130':
            # TMC2130 driver quirks
            self.clear_gstat = False
            cs_actual_mask = self.fields.all_fields[reg_name]["cs_actual"]
        elif name_parts[0] == 'tmc2660':
            # TMC2660 driver quirks
            self.irun_field = "cs"
            reg_name = "READRSP@RDSEL2"
            cs_actual_mask = self.fields.all_fields[reg_name]["se"]
        err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"]
        warn_fields = ["otpw", "t120", "t143", "t150", "t157"]
        for f in err_fields + warn_fields:
            if f in self.fields.all_fields[reg_name]:
                mask |= self.fields.all_fields[reg_name][f]
                if f in err_fields:
                    err_mask |= self.fields.all_fields[reg_name][f]
        self.drv_status_reg_info = [0, reg_name, mask, err_mask, cs_actual_mask]
        # Setup for temperature query
        self.adc_temp = None
        self.adc_temp_reg = self.fields.lookup_register("adc_temp")
        if self.adc_temp_reg is not None:
            pheaters = self.printer.load_object(config, 'heaters')
            pheaters.register_monitor(config)
    def _query_register(self, reg_info, try_clear=False):
        last_value, reg_name, mask, err_mask, cs_actual_mask = reg_info
        cleared_flags = 0
        count = 0
        while 1:
            try:
                val = self.mcu_tmc.get_register(reg_name)
            except self.printer.command_error as e:
                count += 1
                if count < 3 and str(e).startswith("Unable to read tmc uart"):
                    # Allow more retries on a TMC UART read error
                    reactor = self.printer.get_reactor()
                    reactor.pause(reactor.monotonic() + 0.050)
                    continue
                raise
            if val & mask != last_value & mask:
                fmt = self.fields.pretty_format(reg_name, val)
                logging.info("TMC '%s' reports %s", self.stepper_name, fmt)
            reg_info[0] = last_value = val
            if not val & err_mask:
                if not cs_actual_mask or val & cs_actual_mask:
                    break
                irun = self.fields.get_field(self.irun_field)
                if self.check_timer is None or irun < 4:
                    break
                if (self.irun_field == "irun"
                    and not self.fields.get_field("ihold")):
                    break
                # CS_ACTUAL field of zero - indicates a driver reset
            count += 1
            if count >= 3:
                fmt = self.fields.pretty_format(reg_name, val)
                raise self.printer.command_error("TMC '%s' reports error: %s"
                                                 % (self.stepper_name, fmt))
            if try_clear and val & err_mask:
                try_clear = False
                cleared_flags |= val & err_mask
                self.mcu_tmc.set_register(reg_name, val & err_mask)
        return cleared_flags
    def _query_temperature(self):
        try:
            self.adc_temp = self.mcu_tmc.get_register(self.adc_temp_reg)
        except self.printer.command_error as e:
            # Ignore comms error for temperature
            self.adc_temp = None
            return
    def _do_periodic_check(self, eventtime):
        try:
            self._query_register(self.drv_status_reg_info)
            if self.gstat_reg_info is not None:
                self._query_register(self.gstat_reg_info)
            if self.adc_temp_reg is not None:
                self._query_temperature()
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER
        return eventtime + 1.
    def stop_checks(self):
        if self.check_timer is None:
            return
        self.printer.get_reactor().unregister_timer(self.check_timer)
        self.check_timer = None
    def start_checks(self):
        if self.check_timer is not None:
            self.stop_checks()
        cleared_flags = 0
        self._query_register(self.drv_status_reg_info)
        if self.gstat_reg_info is not None:
            cleared_flags = self._query_register(self.gstat_reg_info,
                                                 try_clear=self.clear_gstat)
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        self.check_timer = reactor.register_timer(self._do_periodic_check,
                                                  curtime + 1.)
        if cleared_flags:
            reset_mask = self.fields.all_fields["GSTAT"]["reset"]
            if cleared_flags & reset_mask:
                return True
        return False
    def get_status(self, eventtime=None):
        if self.check_timer is None:
            return {'drv_status': None, 'temperature': None}
        temp = None
        if self.adc_temp is not None:
            temp = round((self.adc_temp - 2038) / 7.7, 2)
        last_value, reg_name = self.drv_status_reg_info[:2]
        if last_value != self.last_drv_status:
            self.last_drv_status = last_value
            fields = self.fields.get_reg_fields(reg_name, last_value)
            self.last_drv_fields = {n: v for n, v in fields.items() if v}
        return {'drv_status': self.last_drv_fields, 'temperature': temp}

######################################################################
# Record driver status
######################################################################

class TMCStallguardDump:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.mcu_tmc = mcu_tmc
        self.mcu = self.mcu_tmc.get_mcu()
        self.fields = self.mcu_tmc.get_fields()
        self.sg2_supp = False
        self.sg4_reg_name = None
        self.batch_bulk = None
        # It is possible to support TMC2660, just disable it for now
        if not self.fields.all_fields.get("DRV_STATUS", None):
            return
        # Collect driver capabilities
        if self.fields.all_fields["DRV_STATUS"].get("sg_result", None):
            self.sg2_supp = True
        # New drivers have separate register for SG4 result
        if self.mcu_tmc.name_to_reg.get("SG_RESULT", 0):
            self.sg4_reg_name = "SG_RESULT"
        # 2240 supports both SG2 & SG4
        if self.sg4_reg_name is None:
            if self.mcu_tmc.name_to_reg.get("SG4_RESULT", 0):
                self.sg4_reg_name = "SG4_RESULT"
        # TMC2208
        if not self.sg2_supp and self.sg4_reg_name is None:
            return
        self.optimized_spi = False
        # Bulk API
        self.samples = []
        self.query_timer = None
        self.error = None
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._dump, self._start, self._stop)
        api_resp = {'header': ('time', 'sg_result', 'cs_actual')}
        self.batch_bulk.add_mux_endpoint("tmc/stallguard_dump", "name",
                                         self.stepper_name, api_resp)
    def can_record_sg2(self):
        return self.sg2_supp and self.batch_bulk is not None
    def _start(self):
        self.error = None
        status = self.mcu_tmc.get_register_raw("DRV_STATUS")
        if status.get("spi_status"):
            self.optimized_spi = True
        reactor = self.printer.get_reactor()
        self.query_timer = reactor.register_timer(self._query_tmc,
                                                  reactor.NOW)
    def _stop(self):
        self.printer.get_reactor().unregister_timer(self.query_timer)
        self.query_timer = None
        self.samples = []
    def _query_tmc(self, eventtime):
        sg_result = -1
        cs_actual = -1
        recv_time = eventtime
        try:
            if self.optimized_spi or self.sg4_reg_name == "SG4_RESULT":
                #TMC2130/TMC5160/TMC2240
                status = self.mcu_tmc.get_register_raw("DRV_STATUS")
                reg_val = status["data"]
                cs_actual = self.fields.get_field("cs_actual", reg_val)
                sg_result = self.fields.get_field("sg_result", reg_val)
                is_stealth = self.fields.get_field("stealth", reg_val)
                recv_time = status["#receive_time"]
                if is_stealth and self.sg4_reg_name == "SG4_RESULT":
                    sg4_ret = self.mcu_tmc.get_register_raw("SG4_RESULT")
                    sg_result = sg4_ret["data"]
                    recv_time = sg4_ret["#receive_time"]
            else:
                # TMC2209
                if self.sg4_reg_name == "SG_RESULT":
                    sg4_ret = self.mcu_tmc.get_register_raw("SG_RESULT")
                    sg_result = sg4_ret["data"]
                    recv_time = sg4_ret["#receive_time"]
        except self.printer.command_error as e:
            self.error = e
            return self.printer.get_reactor().NEVER
        print_time = self.mcu.estimated_print_time(recv_time)
        self.samples.append((print_time, sg_result, cs_actual))
        if self.optimized_spi:
            return eventtime + 0.001
        # UART queried as fast as possible
        return eventtime + 0.005
    def _dump(self, eventtime):
            if self.error:
                raise self.error
            samples = self.samples
            self.samples = []
            return {"data": samples}


######################################################################
# G-Code command helpers
######################################################################

class TMCCommandHelper:
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.config_name = config.get_name()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.current_helper = current_helper
        self.fields = mcu_tmc.get_fields()
        self.stepper = None
        # Stepper phase tracking
        self.mcu_phase_offset = None
        # Stepper enable/disable tracking
        self.toff = None
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.enable_mutex = self.printer.get_reactor().mutex()
        # DUMP_TMC support
        self.read_registers = self.read_translate = None
        # Common tmc helpers
        self.echeck_helper = TMCErrorCheck(config, mcu_tmc)
        self.record_helper = TMCStallguardDump(config, mcu_tmc)
        TMCMicrostepHelper(config, mcu_tmc)
        # Register callbacks
        self.printer.register_event_handler("stepper:sync_mcu_position",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("stepper:set_dir_inverted",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_TMC_FIELD", "STEPPER", self.name,
                                   self.cmd_SET_TMC_FIELD,
                                   desc=self.cmd_SET_TMC_FIELD_help)
        gcode.register_mux_command("INIT_TMC", "STEPPER", self.name,
                                   self.cmd_INIT_TMC,
                                   desc=self.cmd_INIT_TMC_help)
        gcode.register_mux_command("SET_TMC_CURRENT", "STEPPER", self.name,
                                   self.cmd_SET_TMC_CURRENT,
                                   desc=self.cmd_SET_TMC_CURRENT_help)
        # StallGuard2 calibration needs an SGT field and a readable
        # sg_result (TMC2660 has the former but no DRV_STATUS register)
        if self.fields.lookup_register("sgt", None) is None:
            return
        if not self.record_helper.can_record_sg2():
            return
        sconfig = config.getsection(self.stepper_name)
        self.full_steps_per_rotation = sconfig.getint(
            'full_steps_per_rotation', 200, minval=1)
        endstop_pin = sconfig.get('endstop_pin', None, note_valid=False)
        self.has_virtual_endstop = (endstop_pin is not None
                                    and 'virtual_endstop' in endstop_pin)
        gcode.register_mux_command("TMC_CALIBRATE", "STEPPER", self.name,
                                   self.cmd_TMC_CALIBRATE,
                                   desc=self.cmd_TMC_CALIBRATE_help)
    def _init_registers(self, print_time=None):
        # Send registers
        for reg_name in list(self.fields.registers.keys()):
            val = self.fields.registers[reg_name] # Val may change during loop
            self.mcu_tmc.set_register(reg_name, val, print_time)
    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"
    def cmd_INIT_TMC(self, gcmd):
        logging.info("INIT_TMC %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._init_registers(print_time)
    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"
    def cmd_SET_TMC_FIELD(self, gcmd):
        field_name = gcmd.get('FIELD').lower()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int('VALUE', None)
        velocity = gcmd.get_float('VELOCITY', None, minval=0.)
        if (value is None) == (velocity is None):
            raise gcmd.error("Specify either VALUE or VELOCITY")
        if velocity is not None:
            if self.mcu_tmc.get_tmc_frequency() is None:
                raise gcmd.error(
                    "VELOCITY parameter not supported by this driver")
            value = TMCtstepHelper(self.mcu_tmc, velocity,
                                   pstepper=self.stepper)
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)
    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC driver"
    def cmd_SET_TMC_CURRENT(self, gcmd):
        ch = self.current_helper
        prev_cur, prev_hold_cur, req_hold_cur, max_cur = ch.get_current()
        run_current = gcmd.get_float('CURRENT', None, minval=0., maxval=max_cur)
        hold_current = gcmd.get_float('HOLDCURRENT', None,
                                      above=0., maxval=max_cur)
        if run_current is not None or hold_current is not None:
            if run_current is None:
                run_current = prev_cur
            if hold_current is None:
                hold_current = req_hold_cur
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            ch.set_current(run_current, hold_current, print_time)
            prev_cur, prev_hold_cur, req_hold_cur, max_cur = ch.get_current()
        # Report values
        if prev_hold_cur is None:
            gcmd.respond_info("Run Current: %0.2fA" % (prev_cur,))
        else:
            gcmd.respond_info("Run Current: %0.2fA Hold Current: %0.2fA"
                              % (prev_cur, prev_hold_cur))
    # Stepper phase tracking
    def _get_phases(self):
        return (256 >> self.fields.get_field("mres")) * 4
    def get_phase_offset(self):
        return self.mcu_phase_offset, self._get_phases()
    def _query_phase(self):
        field_name = "mscnt"
        if self.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "mstep"
        reg = self.mcu_tmc.get_register(self.fields.lookup_register(field_name))
        return self.fields.get_field(field_name, reg)
    def _handle_sync_mcu_pos(self, stepper):
        if stepper.get_name() != self.stepper_name:
            return
        try:
            driver_phase = self._query_phase()
        except self.printer.command_error as e:
            logging.info("Unable to obtain tmc %s phase", self.stepper_name)
            self.mcu_phase_offset = None
            enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
            if enable_line.is_motor_enabled():
                raise
            return
        if not stepper.get_dir_inverted()[0]:
            driver_phase = 1023 - driver_phase
        phases = self._get_phases()
        phase = int(float(driver_phase) / 1024 * phases + .5) % phases
        moff = (phase - stepper.get_mcu_position()) % phases
        if self.mcu_phase_offset is not None and self.mcu_phase_offset != moff:
            logging.warning("Stepper %s phase change (was %d now %d)",
                            self.stepper_name, self.mcu_phase_offset, moff)
        self.mcu_phase_offset = moff
    # Stepper enable/disable tracking
    def _do_enable(self, print_time):
        if self.toff is not None:
            # Shared enable via comms handling
            self.fields.set_field("toff", self.toff)
        self._init_registers()
        did_reset = self.echeck_helper.start_checks()
        if did_reset:
            self.mcu_phase_offset = None
        # Calculate phase offset
        if self.mcu_phase_offset is not None:
            return
        gcode = self.printer.lookup_object("gcode")
        with gcode.get_mutex():
            if self.mcu_phase_offset is not None:
                return
            logging.info("Pausing toolhead to calculate %s phase offset",
                         self.stepper_name)
            self.printer.lookup_object('toolhead').wait_moves()
            self._handle_sync_mcu_pos(self.stepper)
    def _do_disable(self, print_time):
        if self.toff is not None:
            val = self.fields.set_field("toff", 0)
            reg_name = self.fields.lookup_register("toff")
            self.mcu_tmc.set_register(reg_name, val, print_time)
        self.echeck_helper.stop_checks()
    def _handle_stepper_enable(self, print_time, is_enable):
        def enable_disable_cb(eventtime):
            try:
                with self.enable_mutex:
                    if is_enable:
                        self._do_enable(print_time)
                    else:
                        self._do_disable(print_time)
            except self.printer.command_error as e:
                self.printer.invoke_shutdown(str(e))
        self.printer.get_reactor().register_callback(enable_disable_cb)
    # Initial startup handling
    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        # Note pulse duration and step_both_edge optimizations available
        self.stepper.setup_default_pulse_duration(.000000100, True)
    def _handle_connect(self):
        # Check if using step on both edges optimization
        pulse_duration, step_both_edge = self.stepper.get_pulse_duration()
        if step_both_edge:
            self.fields.set_field("dedge", 1)
        # Check for soft stepper enable/disable
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        enable_line.register_state_callback(self._handle_stepper_enable)
        if not enable_line.has_dedicated_enable():
            self.toff = self.fields.get_field("toff")
            self.fields.set_field("toff", 0)
            logging.info("Enabling TMC virtual enable for '%s'",
                         self.stepper_name)
        # Send init
        try:
            self._init_registers()
        except self.printer.command_error as e:
            logging.info("TMC %s failed to init: %s", self.name, str(e))
    # get_status information export
    def get_status(self, eventtime=None):
        cpos = None
        if self.stepper is not None and self.mcu_phase_offset is not None:
            cpos = self.stepper.mcu_to_commanded_position(self.mcu_phase_offset)
        current = self.current_helper.get_current()
        res = {'mcu_phase_offset': self.mcu_phase_offset,
               'phase_offset_position': cpos,
               'run_current': current[0],
               'hold_current': current[1]}
        res.update(self.echeck_helper.get_status(eventtime))
        return res
    # DUMP_TMC support
    def setup_register_dump(self, read_registers, read_translate=None):
        self.read_registers = read_registers
        self.read_translate = read_translate
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("DUMP_TMC", "STEPPER", self.name,
                                   self.cmd_DUMP_TMC,
                                   desc=self.cmd_DUMP_TMC_help)
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, gcmd):
        logging.info("DUMP_TMC %s", self.name)
        reg_name = gcmd.get('REGISTER', None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            val = self.fields.registers.get(reg_name)
            if (val is not None) and (reg_name not in self.read_registers):
                # write-only register
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            elif reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            gcmd.respond_info("========== Write-only registers ==========")
            for reg_name, val in self.fields.registers.items():
                if reg_name not in self.read_registers:
                    gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in self.read_registers:
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
    cmd_TMC_CALIBRATE_help = "Calibrate TMC StallGuard2 parameters"
    def cmd_TMC_CALIBRATE(self, gcmd):
        target = gcmd.get('TARGET')
        if target not in ("sgt", "sgt_velocity", "verify"):
            raise gcmd.error("Unknown target name '%s'" % (target,))
        TMCStallGuardHelper(self, gcmd).start(gcmd)


######################################################################
# StallGuard2 calibration
######################################################################

SGT_MIN, SGT_MAX = -64, 63
# Motor shaft speed window for the TARGET=sgt search
SGT_SEARCH_MIN_RPM, SGT_SEARCH_MAX_RPM = 2., 10.
# A decision is taken over three electrical periods of motor travel
SG_WINDOW_FULLSTEPS = 12
# sg_result only reflects a register change after the next electrical
# period (four full steps) has been driven
SG_SETTLE_FULLSTEPS = 4

class TMCStallGuardHelper:
    def __init__(self, cmdhelper, gcmd):
        self.printer = cmdhelper.printer
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.mcu_tmc = cmdhelper.mcu_tmc
        self.fields = self.mcu_tmc.get_fields()
        self.config_name = cmdhelper.config_name
        self.stepper_name = cmdhelper.stepper_name
        self.short_name = cmdhelper.name
        self.has_virtual_endstop = cmdhelper.has_virtual_endstop
        self.record_helper = cmdhelper.record_helper
        self.current_helper = cmdhelper.current_helper
        self.respond_info = gcmd.respond_info
        self.target = gcmd.get('TARGET')
        fmove = self.printer.lookup_object('force_move')
        self.mcu_stepper = fmove.lookup_stepper(self.stepper_name)
        self.mcu = self.mcu_stepper.get_mcu()
        self.step_dist = self.mcu_stepper.get_step_dist()
        # Travel is measured in motor microsteps.  StallGuard follows the
        # motor's electrical period, so gear_ratio and
        # full_steps_per_rotation must not leak in: get_rotation_distance()
        # counts steps per *output* rotation and is off by both.
        microsteps = 256 >> self.fields.get_field("mres")
        full_steps = cmdhelper.full_steps_per_rotation
        self.window_steps = SG_WINDOW_FULLSTEPS * microsteps
        self.settle_steps = SG_SETTLE_FULLSTEPS * microsteps
        steps_per_rev = full_steps * microsteps
        self.min_sps = steps_per_rev * SGT_SEARCH_MIN_RPM / 60.
        self.max_sps = steps_per_rev * SGT_SEARCH_MAX_RPM / 60.
        self.min_sg = gcmd.get_int('MIN_SG', 16, minval=1)
        # Samples are (print_time, mcu_position, sg_result)
        self.msgs = []
        self.samples = []
        self.settle_pos = None
        self.is_running = False
        self._timer = None
        self.last_notice = 0.
        self.max_sg = 0
        # Register save/restore
        self._dirty_regs = collections.OrderedDict()
        self._prev_state = collections.OrderedDict()
        # TARGET=sgt search state
        self.sgt = 0
        self.positive_at = None
    def start(self, gcmd):
        # On TMC2240 a non-zero driver_SG4_THRS makes sensorless homing
        # use StallGuard4 and ignore driver_SGT entirely
        if self.fields.lookup_register("sg4_thrs", None) is not None:
            if self.fields.get_field("sg4_thrs"):
                self.respond_info(
                    "Warning: driver_SG4_THRS is set - sensorless homing"
                    " uses StallGuard4 and ignores driver_SGT")
        if self.target == "verify":
            self._run_verify(gcmd)
            return
        self.toolhead.wait_moves()
        # Interactive targets hold the global ABORT command while they
        # run, the same way manual_probe and bed_screws do, which also
        # keeps two calibrations from running at once
        try:
            self.gcode.register_command("ABORT", self.cmd_ABORT,
                                        desc=self.cmd_ABORT_help)
        except self.printer.config_error:
            raise gcmd.error("Another calibration in progress")
        self.is_running = True
        # StallGuard2 needs SpreadCycle and CoolStep off (AN-002 section
        # 2: SEMIN=0 while parameterizing), and no high velocity mode.
        # The user's sfilt is left alone: the threshold has to be found
        # under the filter setting homing will run with, and AN-002 2.2
        # recommends filtering off for stall detection.
        if self.fields.lookup_register("en_pwm_mode", None) is not None:
            self._set_field("en_pwm_mode", 0)
        if self.fields.lookup_register("semin", None) is not None:
            self._set_field("semin", 0)
        if self.fields.lookup_register("thigh", None) is not None:
            self._set_field("thigh", 0)
        if self.target == "sgt":
            self._set_field("sgt", self.sgt)
        self._send_fields()
        self._mark_settle()
        if self.target == "sgt":
            self.respond_info(
                "Move the stepper/toolhead by hand or with FORCE_MOVE at a"
                " steady %.1f - %.1f mm/s (%d - %d motor RPM)\n"
                "driver_SGT is raised until sg_result leaves zero, then the"
                " boundary is confirmed from above" % (
                    self.min_sps * self.step_dist,
                    self.max_sps * self.step_dist,
                    SGT_SEARCH_MIN_RPM, SGT_SEARCH_MAX_RPM))
        else:
            self.respond_info(
                "Move the stepper/toolhead, slowly increasing the velocity"
                " (driver_SGT=%d, looking for sg_result >= %d)" % (
                    self.fields.get_field("sgt"), self.min_sg))
        self.respond_info("Use ABORT to exit")
        self.record_helper.batch_bulk.add_client(self.handle_batch)
        self._timer = self.reactor.register_timer(self._event,
                                                  self.reactor.NOW)
    cmd_ABORT_help = "Abort StallGuard calibration"
    def cmd_ABORT(self, gcmd):
        self._finish()
        gcmd.respond_info("StallGuard calibration aborted")
    def _finish(self):
        self.is_running = False
        self.gcode.register_command("ABORT", None)
        if self._timer is not None:
            self.reactor.unregister_timer(self._timer)
            self._timer = None
        self.toolhead.wait_moves()
        self._restore_fields()
    # Register helpers
    def _set_field(self, field_name, value):
        self._prev_state[field_name] = self.fields.get_field(field_name)
        reg_name = self.fields.lookup_register(field_name)
        self._dirty_regs[reg_name] = self.fields.set_field(field_name, value)
    def _send_fields(self):
        for reg, val in self._dirty_regs.items():
            self.mcu_tmc.set_register(reg, val)
        self._dirty_regs.clear()
    def _restore_fields(self):
        for field, val in list(self._prev_state.items()):
            self._set_field(field, val)
        self._send_fields()
        self._prev_state.clear()
    def _set_sgt(self, sgt):
        self.sgt = sgt
        reg_name = self.fields.lookup_register("sgt")
        self._dirty_regs[reg_name] = self.fields.set_field("sgt", sgt)
        self._send_fields()
        self._mark_settle()
        self.respond_info("Testing driver_SGT=%d" % (sgt,))
    def _mark_settle(self):
        # Ignore readings until the motor has moved one electrical period
        # past the register write
        now = self.mcu.estimated_print_time(self.reactor.monotonic())
        self.settle_pos = self.mcu_stepper.get_past_mcu_position(now)
        self.samples = []
    # Sample handling
    def handle_batch(self, msg):
        self.msgs.append(msg)
        return self.is_running
    def _ingest(self):
        new = []
        while self.msgs:
            for ptime, sg, _ in self.msgs.pop(0)["data"]:
                if sg < 0:
                    # TMCStallguardDump could not decode a result
                    return None
                pos = self.mcu_stepper.get_past_mcu_position(ptime)
                new.append((ptime, pos, sg))
        return new
    def _notice(self, msg):
        now = self.reactor.monotonic()
        if now - self.last_notice < 5.:
            return
        self.last_notice = now
        self.respond_info(msg)
    def _event(self, eventtime):
        new = self._ingest()
        if new is None:
            self.respond_info("Unable to read sg_result from driver"
                              " - aborting")
            self._finish()
            return self.reactor.NEVER
        if self.settle_pos is not None:
            new = [s for s in new
                   if abs(s[1] - self.settle_pos) >= self.settle_steps]
            if new:
                self.settle_pos = None
        if new:
            self.max_sg = max(self.max_sg, max([sg for _, _, sg in new]))
            self.samples.extend(new)
            # Keep enough history for a window at the slowest speed
            keep = 2. * self.window_steps / self.min_sps
            cutoff = self.samples[-1][0] - keep
            self.samples = [s for s in self.samples if s[0] >= cutoff]
        if self.target == "sgt":
            done = self._sgt_step()
        else:
            done = self._velocity_step()
        if done:
            self._finish()
            return self.reactor.NEVER
        return eventtime + 0.5
    def _find_window(self, accept, max_sps=None):
        # Return the oldest run of samples spanning window_steps of travel
        # at an acceptable speed for which accept() holds, as (W, sps)
        samples = self.samples
        n = len(samples)
        R = 1
        for L in range(n):
            t0, p0, _ = samples[L]
            if R <= L:
                R = L + 1
            while R < n and abs(samples[R][1] - p0) < self.window_steps:
                R += 1
            if R >= n:
                # Not enough travel recorded past this point yet
                return None
            t1, p1, _ = samples[R]
            if t1 <= t0:
                continue
            sps = abs(p1 - p0) / (t1 - t0)
            if sps < self.min_sps:
                continue
            if max_sps is not None and sps > max_sps:
                self._notice("Too fast: %.1f mm/s, stay below %.1f mm/s" % (
                    sps * self.step_dist, max_sps * self.step_dist))
                continue
            W = samples[L:R+1]
            if accept(W):
                return W, sps
        return None
    # TARGET=sgt: find the highest SGT that still reads zero at low speed
    def _sgt_step(self):
        def decisive(W):
            sgs = [sg for _, _, sg in W]
            return min(sgs) > 0 or max(sgs) == 0
        found = self._find_window(decisive, self.max_sps)
        if found is None:
            return False
        W, sps = found
        if min([sg for _, _, sg in W]) > 0:
            self.positive_at = self.sgt
            if self.sgt <= SGT_MIN:
                self.respond_info("sg_result stays positive down to SGT %d"
                                  " - motor too fast?" % (SGT_MIN,))
                return True
            self._set_sgt(self.sgt - 1)
            return False
        # The window read zero throughout
        if self.positive_at == self.sgt + 1:
            self._finish_sgt(sps)
            return True
        if self.sgt >= SGT_MAX:
            self.respond_info("sg_result stays zero up to SGT %d"
                              " - motor too slow or overloaded?" % (SGT_MAX,))
            return True
        self._set_sgt(self.sgt + 1)
        return False
    def _finish_sgt(self, sps):
        run_current = self.current_helper.get_current()[0]
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.config_name, 'driver_SGT', self.sgt)
        self.respond_info(
            "driver_SGT: %d (sg_result zero at %.1f mm/s, positive at %d)\n"
            "Measured with run_current %.3fA and sfilt=%d; SGT only holds"
            " near the operating point it was found at (AN-002 2.5.1)\n"
            "Next: SAVE_CONFIG, then TMC_CALIBRATE STEPPER=%s"
            " TARGET=sgt_velocity and TARGET=verify" % (
                self.sgt, sps * self.step_dist, self.positive_at,
                run_current, self.fields.get_field("sfilt"),
                self.short_name))
        self.respond_info(
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer.")
    # TARGET=sgt_velocity: lowest speed at which sg_result is usable
    def _velocity_step(self):
        min_sg = self.min_sg
        def usable(W):
            return min([sg for _, _, sg in W]) >= min_sg
        found = self._find_window(usable)
        if found is None:
            return False
        W, sps = found
        vel = sps * self.step_dist
        lowest = min([sg for _, _, sg in W])
        self.respond_info(
            "sg_result stays >= %d from %.1f mm/s (lowest %d, peak seen %d)"
            % (self.min_sg, vel, lowest, self.max_sg))
        if self.fields.lookup_register("tcoolthrs", None) is None:
            return True
        # AN-002 2.1: the stall velocity gate should sit close to the
        # working velocity, because back-EMF changes quickly during
        # acceleration and can trip StallGuard early.  Without a
        # coolstep_threshold, Klipper arms the DIAG output for the whole
        # homing ramp.
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.config_name, 'coolstep_threshold', "%.1f" % (vel,))
        self.respond_info(
            "coolstep_threshold: %.1f mm/s - stall detection is armed only"
            " above this speed; home faster than it" % (vel,))
        self.respond_info(
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer.")
        return True
    # TARGET=verify: record sg_result through a real sensorless home
    def _default_axis(self):
        kin = self.toolhead.get_kinematics()
        rails = getattr(kin, 'rails', None)
        if not rails:
            return None
        for i, rail in enumerate(rails[:3]):
            names = [s.get_name() for s in rail.get_steppers()]
            if self.stepper_name in names:
                return "xyz"[i]
        return None
    def _run_verify(self, gcmd):
        if not self.has_virtual_endstop:
            raise gcmd.error("%s does not home with a tmc virtual_endstop"
                             % (self.stepper_name,))
        axis = gcmd.get('AXIS', self._default_axis())
        if axis is None:
            raise gcmd.error("Unable to determine the homing axis of %s"
                             " - specify AXIS=" % (self.stepper_name,))
        self.toolhead.wait_moves()
        self.is_running = True
        self.record_helper.batch_bulk.add_client(self.handle_batch)
        try:
            self.gcode.run_script_from_command("G28 %s" % (axis.upper(),))
        finally:
            self.toolhead.wait_moves()
            # Let the last batch arrive before dropping the client
            self.reactor.pause(self.reactor.monotonic()
                               + 2. * bulk_sensor.BATCH_INTERVAL)
            self.is_running = False
        samples = self._ingest()
        if samples is None:
            raise gcmd.error("Unable to read sg_result from driver")
        self._report_verify(samples)
    def _report_verify(self, samples):
        # Split the recording into moves (runs of samples with steps)
        moves = []
        cur = []
        for ptime, pos, sg in samples:
            if pos != self.mcu_stepper.get_past_mcu_position(ptime - 0.02):
                cur.append((ptime, pos, sg))
            elif cur:
                moves.append(cur)
                cur = []
        if cur:
            moves.append(cur)
        min_travel = 2 * self.settle_steps + self.window_steps
        count = 0
        for m in moves:
            p0, pn = m[0][1], m[-1][1]
            travel = abs(pn - p0)
            if travel < min_travel:
                continue
            count += 1
            vel = travel / (m[-1][0] - m[0][0]) * self.step_dist
            # Cruise excludes the ramp-in and the final period, where a
            # stall collapses sg_result by design
            cruise = sorted([sg for _, pos, sg in m
                             if abs(pos - p0) >= self.settle_steps
                             and abs(pn - pos) >= self.settle_steps])
            final = [sg for _, pos, sg in m
                     if abs(pn - pos) < self.settle_steps]
            plateau = cruise[len(cruise) // 2]
            cruise_min = cruise[0]
            end_min = min(final)
            self.respond_info(
                "Move %d: %.1f mm/s, sg_result plateau %d, cruise minimum"
                " %d, end %d%s" % (count, vel, plateau, cruise_min, end_min,
                                  " (stall)" if end_min == 0 else ""))
            if end_min != 0:
                continue
            # AN-002 2.2: the lowest reading preceding the stall is the
            # safety margin against false stall detection
            if plateau < 50:
                self.respond_info(
                    "  Unloaded sg_result is only %d at %.1f mm/s: little"
                    " headroom, raise driver_SGT or the homing speed"
                    % (plateau, vel))
            elif cruise_min * 5 < plateau:
                self.respond_info(
                    "  sg_result dipped to %d%% of the plateau during travel"
                    " (resonance or a tight spot): close to a false stall"
                    % (100 * cruise_min // plateau,))
            else:
                self.respond_info(
                    "  Lowest reading before the stall keeps %d%% of the"
                    " plateau" % (100 * cruise_min // plateau,))
        if not count:
            self.respond_info("No homing move long enough to evaluate"
                              " was recorded")
        if self.fields.lookup_register("tcoolthrs", None) is not None:
            if not self.fields.get_field("tcoolthrs"):
                self.respond_info(
                    "coolstep_threshold is not set: stall detection was"
                    " armed during the whole acceleration ramp. Run"
                    " TARGET=sgt_velocity to set it (AN-002 2.1)")


######################################################################
# TMC virtual pins
######################################################################

# Helper class for "sensorless homing"
class TMCVirtualPinHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        if self.fields.lookup_register('diag0_stall') is not None:
            if config.get('diag0_pin', None) is not None:
                self.diag_pin = config.get('diag0_pin')
                self.diag_pin_field = 'diag0_stall'
            else:
                self.diag_pin = config.get('diag1_pin', None)
                self.diag_pin_field = 'diag1_stall'
        else:
            self.diag_pin = config.get('diag_pin', None)
            self.diag_pin_field = None
        self.mcu_endstop = None
        self._dirty_regs = collections.OrderedDict()
        self._prev_state = collections.OrderedDict()
        # Register virtual_endstop pin
        name_parts = config.get_name().split()
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)
    def setup_pin(self, pin_type, pin_params):
        # Validate pin
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise ppins.error("tmc virtual endstop only useful as endstop")
        if pin_params['invert'] or pin_params['pullup']:
            raise ppins.error("Can not pullup/invert tmc virtual pin")
        if self.diag_pin is None:
            raise ppins.error("tmc virtual endstop requires diag pin config")
        # Setup for sensorless homing
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self.handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self.handle_homing_move_end)
        self.mcu_endstop = ppins.setup_pin('endstop', self.diag_pin)
        return self.mcu_endstop
    def _set_field(self, field_name, value):
        self._prev_state[field_name] = self.fields.get_field(field_name)
        reg_name = self.fields.lookup_register(field_name)
        self._dirty_regs[reg_name] = self.fields.set_field(field_name, value)
    def _send_fields(self):
        for reg, val in self._dirty_regs.items():
            self.mcu_tmc.set_register(reg, val)
        self._dirty_regs.clear()
    def handle_homing_move_begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        sg4_thrs = 0
        if self.fields.lookup_register("sg4_thrs", None) is not None:
            sg4_thrs = self.fields.get_field("sg4_thrs")
        # Enable/disable stealthchop
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is None:
            # On "stallguard4" drivers, "stealthchop" must be enabled
            self._set_field("tpwmthrs", 0)
            self._set_field("en_spreadcycle", 0)
        elif sg4_thrs:
            # TMC2240 using SG4, "stealthchop" must be enabled
            self._set_field("en_pwm_mode", 1)
            self._set_field("tpwmthrs", 0)
            self._set_field(self.diag_pin_field, 1)
        else:
            # On earlier drivers, "stealthchop" must be disabled
            self._set_field("en_pwm_mode", 0)
            self._set_field(self.diag_pin_field, 1)
        # Enable tcoolthrs (if not already)
        if self.fields.get_field("tcoolthrs") == 0:
            self._set_field("tcoolthrs", 0xfffff)
        # Disable thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            self._set_field("thigh", 0)
        self._send_fields()
    def handle_homing_move_end(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Restore previous state
        for field, val in list(self._prev_state.items()):
            self._set_field(field, val)
        self._send_fields()
        self._prev_state.clear()


######################################################################
# Config reading helpers
######################################################################

# Helper to initialize the wave table from config or defaults
def TMCWaveTableHelper(config, mcu_tmc):
    set_config_field = mcu_tmc.get_fields().set_config_field
    set_config_field(config, "mslut0", 0xAAAAB554)
    set_config_field(config, "mslut1", 0x4A9554AA)
    set_config_field(config, "mslut2", 0x24492929)
    set_config_field(config, "mslut3", 0x10104222)
    set_config_field(config, "mslut4", 0xFBFFFFFF)
    set_config_field(config, "mslut5", 0xB5BB777D)
    set_config_field(config, "mslut6", 0x49295556)
    set_config_field(config, "mslut7", 0x00404222)
    set_config_field(config, "w0", 2)
    set_config_field(config, "w1", 1)
    set_config_field(config, "w2", 1)
    set_config_field(config, "w3", 1)
    set_config_field(config, "x1", 128)
    set_config_field(config, "x2", 255)
    set_config_field(config, "x3", 255)
    set_config_field(config, "start_sin", 0)
    set_config_field(config, "start_sin90", 247)

# Helper to configure the microstep settings
def TMCMicrostepHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    stepper_name = " ".join(config.get_name().split()[1:])
    if not config.has_section(stepper_name):
        raise config.error(
            "Could not find config section '[%s]' required by tmc driver"
            % (stepper_name,))
    sconfig = config.getsection(stepper_name)
    steps = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
    mres = sconfig.getchoice('microsteps', steps)
    fields.set_field("mres", mres)
    fields.set_field("intpol", config.getboolean("interpolate", True))

# Helper for calculating TSTEP based values from velocity
def TMCtstepHelper(mcu_tmc, velocity, pstepper=None, config=None):
    if velocity <= 0.:
        return 0xfffff
    if pstepper is not None:
        step_dist = pstepper.get_step_dist()
    else:
        stepper_name = " ".join(config.get_name().split()[1:])
        sconfig = config.getsection(stepper_name)
        rotation_dist, steps_per_rotation = stepper.parse_step_distance(sconfig)
        step_dist = rotation_dist / steps_per_rotation
    mres = mcu_tmc.get_fields().get_field("mres")
    step_dist_256 = step_dist / (1 << mres)
    tmc_freq = mcu_tmc.get_tmc_frequency()
    threshold = int(tmc_freq * step_dist_256 / velocity + .5)
    return max(0, min(0xfffff, threshold))

# Helper to configure stealthChop-spreadCycle transition velocity
def TMCStealthchopHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    en_pwm_mode = False
    velocity = config.getfloat('stealthchop_threshold', None, minval=0.)
    tpwmthrs = 0xfffff

    if velocity is not None:
        en_pwm_mode = True
        tpwmthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tpwmthrs", tpwmthrs)

    reg = fields.lookup_register("en_pwm_mode", None)
    if reg is not None:
        fields.set_field("en_pwm_mode", en_pwm_mode)
    else:
        # TMC2208 uses en_spreadCycle
        fields.set_field("en_spreadcycle", not en_pwm_mode)

# Helper to configure StallGuard and CoolStep minimum velocity
def TMCVcoolthrsHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('coolstep_threshold', None, minval=0.)
    tcoolthrs = 0
    if velocity is not None:
        tcoolthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tcoolthrs", tcoolthrs)

# Helper to configure StallGuard and CoolStep maximum velocity and
# SpreadCycle-FullStepping (High velocity) mode threshold.
def TMCVhighHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('high_velocity_threshold', None, minval=0.)
    thigh = 0
    if velocity is not None:
        thigh = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("thigh", thigh)
