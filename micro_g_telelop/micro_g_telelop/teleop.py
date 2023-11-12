import keyboard


def main():
    try:
        while True:
            if keyboard.is_pressed("a"):
                # TODO(evan): start experiment
                ...
            elif keyboard.is_pressed("s"):
                # TODO(evan): reset experiment
                ...
            elif keyboard.is_pressed("d"):
                # TODO(evan): disable robot
                ...
    except KeyboardInterrupt:
        ...
