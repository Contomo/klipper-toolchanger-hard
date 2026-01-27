
import logging

TIMER_DT = 0.01

class ProbeBlindButton:
    def __init__(self, printer, on_change, settle_time=2.0):
        if not callable(on_change):
            raise TypeError("ProbeBlindButton requires on_change(old_key, new_key)")
        if printer.lookup_object('gcode').is_printer_ready:
            raise Exception("ProbeBlindButton must be constructed before the printer is READY")

        self.printer          = printer
        self.reactor          = printer.get_reactor()
        self._on_change       = on_change
        self._settle          = float(settle_time)

        self.toolhead         = None
        self._probe_session   = None
        self._probe_obj       = None

        self._busy            = 0
        self._latched         = False
        self._latched_key     = None
        self._timer           = None
        self._settle_deadline = 0.0

        self._stable_key      = None
        self._raw_key         = None

        self.printer.register_event_handler('klippy:connect',
                                            self._on_connect)
        self.printer.register_event_handler('homing:homing_move_begin',
                                            self._on_hmove_begin)
        self.printer.register_event_handler('homing:homing_move_end',
                                            self._on_hmove_end)

    def _on_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self._probe_obj = self.printer.lookup_object('probe', None)
        self._probe_session = getattr(self._probe_obj, 'probe_session', None) if self._probe_obj else None

    def _session_busy(self):
        ps = self._probe_session
        if ps and getattr(ps, 'hw_probe_session', None):
            return True
        prb = self._probe_obj
        if not prb:
            return False
        if getattr(prb, 'multi_probe_pending', False):
            return True
        active = getattr(prb, 'active_probe', None)
        if active is not None:
            aps = getattr(active, 'probe_session', None)
            if aps and getattr(aps, 'multi_probe_pending', False):
                return True
            if aps and getattr(aps, 'hw_probe_session', None):
                return True
        if getattr(prb, '_active_session', None) is not None:
            return True
        return False

    @staticmethod
    def _is_probe_endstop_listed(hmove):
        try:
            return any(hasattr(es, 'probe_prepare') and hasattr(es, 'probe_finish')
                       for es in hmove.get_mcu_endstops())
        except Exception:
            return False

    def _on_hmove_begin(self, hmove):
        if self._is_probe_endstop_listed(hmove):
            self._busy += 1

    def _on_hmove_end(self, hmove):
        if self._is_probe_endstop_listed(hmove) and self._busy > 0:
            self._busy -= 1

    def note_change(self, key):
        """Feed raw key changes here (any comparable object)."""
        now = self.reactor.monotonic()

        old, mid, new = self._stable_key, self._raw_key, key
        self._raw_key = new  # so the timer always sees the freshest value

        if self._timer is None and new == old:
            return
        
        if self._latched:
            if new == old: # reverted back before settle: cancel, no emits
                self._cancel_timer()
                return

        if self._busy > 0 or self._session_busy():
            if not self._latched and new != old: # first change while busy: latch and start timer
                self._latched = True
                self._latched_key = new
                if self._timer is None:
                    self._timer = self.reactor.register_timer(self._timer_cb)
                self.reactor.update_timer(self._timer, now + TIMER_DT)
                return
            # second, different change before settle while still busy:
            # emit old->mid, then mid->new, then reset
            if new != mid and new != old and self._latched:
                self._on_change(old, mid)
                self._on_change(mid, new)
                self._stable_key = new
                self._cancel_timer()
            return

        if new != old:
            self._stable_key = new
            self._on_change(old, new)

    # timers

    def _timer_cb(self, eventtime):
        if not self._latched:
            self._cancel_timer()
            return self.reactor.NEVER
        # still probing/homing: keep blind, clear settle start
        if self._busy > 0 or self._session_busy():
            self._settle_deadline = 0.0
            return eventtime + TIMER_DT
        # just left probing/homing: start settle window
        if self._settle_deadline == 0.0:
            self._settle_deadline = eventtime + self._settle
            return eventtime + TIMER_DT
        # settling
        if eventtime < self._settle_deadline:
            return eventtime + TIMER_DT
        # settle expired
        old, mid, fin = self._stable_key, self._latched_key, self._raw_key
        if fin == old:
            pass # we good
        elif mid == fin:
            self._emit_once(old, fin)
        else:
            self._emit_once(old, mid)
            self._emit_once(mid, fin)
        self._stable_key = fin
        self._cancel_timer()
        return self.reactor.NEVER

    def _cancel_timer(self):
        """Stop timer and clear latch-related state."""
        if self._timer is not None:
            self.reactor.update_timer(self._timer, self.reactor.NEVER)
            self._timer = None
        self._latched = False
        self._latched_key = None
        self._settle_deadline = 0.0

    def _emit_once(self, old, new):
        if old == new:
            return
        try:
            self._on_change(old, new)
        except Exception:
            logging.exception('exception in ProbeBlindButton callback')
