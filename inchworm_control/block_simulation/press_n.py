import pyautogui
import time

# Enable the fail-safe feature by moving the mouse to the top-left corner
pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.0

try:
    while True:
        # pyautogui.keyDown('n')  # Press the "n" key down
        # pyautogui.keyUp('n')  # Release the "n" key
        pyautogui.press('n')
except KeyboardInterrupt:
    print("Program exited.")
