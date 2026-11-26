"""Voice target selection (AI-1): say "track forceps" to select a target, "stop tracking" to clear it.

Recognition runs offline (Vosk) and is limited to the command phrases, so other speech comes out as
"[unk]". A stretch of speech between pauses counts only if it is exactly one command, so a sentence
like "now we'll track the forceps" is ignored. Audio is handled in memory 0.1 s at a time and is
never saved or sent anywhere (DI-22).

For December this runs inside the ADT app. It only produces class names, so it can move into the
auto-input node unchanged once the target selection message exists.
"""
import json
import queue
import threading
import time

CLEAR = "stop tracking"
RATE = 16000  # samples per second; what the Vosk model expects
BLOCK = 1600  # samples per read: 0.1 s


def commands(targets):
    """{phrase: class} for targets {spoken name: class}, plus the clear phrase mapped to None."""
    phrases = {f"track {spoken}": label for spoken, label in targets.items()}
    phrases[CLEAR] = None
    return phrases


class Listener:
    """Listens to the default microphone on a background thread.

    Recognized commands arrive on .commands as (class or None, phrase, speech_end_ns): the class to
    select (None clears), the phrase heard, and when the speaker stopped (DI-7 timing).
    Set .muted to ignore the microphone; .warning holds a problem to show the operator.
    """

    def __init__(self, model_dir, targets):
        # Imported here so the app still runs, keyboard only, if voice isn't installed.
        import sounddevice
        from vosk import KaldiRecognizer, Model, SetLogLevel

        SetLogLevel(-1)  # Vosk logs every model load to stderr
        self.phrases = commands(targets)
        self.recognizer = KaldiRecognizer(Model(model_dir), RATE, json.dumps([*self.phrases, "[unk]"]))
        self.recognizer.SetWords(True)  # word end times, for speech_end_ns
        self.stream = sounddevice.RawInputStream(samplerate=RATE, blocksize=BLOCK, channels=1, dtype="int16")
        self.commands = queue.Queue()
        self.muted, self.warning, self.running = False, None, True
        threading.Thread(target=self._run, daemon=True).start()

    def close(self):
        self.running = False

    def _run(self):
        fed, heard_sound = 0, False  # samples given to the recognizer (its word times count these)
        try:
            with self.stream:
                while self.running:
                    data, _overflowed = self.stream.read(BLOCK)
                    read_ns = time.time_ns()
                    # Muted: feed silence instead, so a half-said phrase ends and the timing stays aligned.
                    data = bytes(len(data)) if self.muted else bytes(data)
                    fed += BLOCK
                    heard_sound = heard_sound or any(data)
                    if not heard_sound and not self.muted and fed == 2 * RATE:
                        # macOS gives silence, not an error, when the terminal lacks microphone permission.
                        self.warning = "mic is silent: check microphone permission"
                    elif heard_sound:
                        self.warning = None
                    if self.recognizer.AcceptWaveform(data):
                        result = json.loads(self.recognizer.Result())
                        if result.get("text") in self.phrases and result.get("result"):
                            end_s = result["result"][-1]["end"]  # seconds into the audio fed so far
                            speech_end_ns = read_ns - int((fed / RATE - end_s) * 1e9)
                            self.commands.put((self.phrases[result["text"]], result["text"], speech_end_ns))
        except Exception as e:  # e.g. the microphone was unplugged; the keyboard still works
            self.warning = f"voice stopped: {e}"
