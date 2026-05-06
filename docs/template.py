"""template

这里是主逻辑的标准模版
"""

from __future__ import annotations

from BasicFuc import *


def main():
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                # Put your main logic here.
                pass
            time.sleep(0.03)
    finally:
        UnitedPosition.Stop()


if __name__ == "__main__":
    main()
