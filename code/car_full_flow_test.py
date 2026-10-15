"""Car-only field mission listener: real motion and dual SG90 payload; no drone commands.

Waits for the existing ground-station RUN command; keeps the normal countdown,
SLAM restart, OCR, two-target route, alignment, approach, return and finish.
"""

from competition_boot import main as boot_main


def main():
    return boot_main(car_only_test=True)


if __name__ == "__main__":
    main()
