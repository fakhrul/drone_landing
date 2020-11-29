# in command prompt, type "pip install pynput" to install pynput.
from pynput.keyboard import Key, Controller

keyboard = Controller()
key = "a"

keyboard.press(key)
keyboard.release(key)