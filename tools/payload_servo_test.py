#!/usr/bin/env python3
"""Stationary four SG90 test: open all/0/1/2/3 or close all; holds PWM on exit."""
import argparse
import logging
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code'))
from components.payload_servos import open_payload, close_payload


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('open', 'close'))
    parser.add_argument('slot', choices=('0', '1', '2', '3', 'all'))
    args = parser.parse_args(argv)
    if args.action == 'close' and args.slot != 'all':
        parser.error('close supports all; use open 0/1/2/3 for individual release')
    logging.basicConfig(level=logging.INFO)
    try:
        ok = (close_payload() if args.action == 'close' else
              open_payload(None if args.slot == 'all' else int(args.slot)))
        if ok is False:
            logging.error('payload PWM command incomplete')
            return 1
        logging.info('PWM command written: %s %s, %dus; no mechanical feedback',
                     args.action, args.slot, 1000 if args.action == 'open' else 1500)
        return 0
    except KeyboardInterrupt:
        try:
            if open_payload() is False:
                logging.error('interrupt payload OPEN incomplete')
        except Exception:
            logging.exception('interrupt payload OPEN failed')
        return 130
    except Exception:
        logging.exception('payload PWM command failed')
        try:
            if open_payload() is False:
                logging.error('failure payload OPEN incomplete')
        except Exception:
            logging.exception('failure payload OPEN failed')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
