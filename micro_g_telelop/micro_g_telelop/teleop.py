import keyboard


def main():
    try:
        while True:
            if keyboard.is_pressed("d"):
                # TODO(evan): disable
                ...
            elif keyboard.is_pressed("e"):
                # TODO(evan): enable
                ...
            elif keyboard.is_pressed("r"):
                # TODO(evan): reset
                ...
    except KeyboardInterrupt:
        ...
