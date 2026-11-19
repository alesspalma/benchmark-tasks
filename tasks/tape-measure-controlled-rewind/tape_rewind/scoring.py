"""Judge the visible travel and final support independently of arm assignment."""

from . import config as C


class Scorer:
    def reset(self, extension):
        self.initial = extension
        self.previous = extension
        self.peak_frame_m = 0.
        self.hold_frames = 0

    def evaluate(self, state):
        extension = state["extension_m"]
        self.peak_frame_m = max(self.peak_frame_m, self.previous - extension)
        self.previous = extension

        snap = self.peak_frame_m > C.SCORE_REWIND_PER_FRAME
        stable = state["case_supported"] and state["case_speed_m_s"] < .025
        seated = extension <= C.HOME_GAP
        hands_clear = state["clearance_m"] >= .055
        if seated and stable and hands_clear and not snap:
            self.hold_frames += 1
        else:
            self.hold_frames = 0

        rewound = self.initial - extension
        score = 0.
        if not snap and stable:
            if self.hold_frames >= 10:
                score = 1.
            elif rewound >= .20:
                score = .75
            elif rewound >= .125:
                score = .50
            elif rewound >= .060:
                score = .25
        return {
            "score": score, "success": score == 1., "hook_seated": seated,
            "extension_mm": round(1000. * extension, 3),
            "peak_frame_travel_mm": round(1000. * self.peak_frame_m, 3),
            "snap_back": snap, "case_supported": state["case_supported"],
            "hands_clear": hands_clear,
            "button_pressed_mm": round(1000. * state["button_press_m"], 3),
        }
