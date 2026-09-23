#!/usr/bin/env python3
import argparse
import time


DEFAULT_BAUD = 9600
SEND_INTERVAL_SECONDS = 2.0


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send numbered test lines through an E220 serial connection.'
    )
    parser.add_argument('--port', required=True, help='Serial device, e.g. /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help='UART baud rate')
    return parser.parse_args()


def format_message(sequence):
    return f'HELLO_LORA,{sequence}'


def main():
    args = parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            'pyserial is required. Install it with: python3 -m pip install pyserial'
        ) from exc

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as connection:
            sequence = 1
            while True:
                message = format_message(sequence)
                connection.write(f'{message}\n'.encode('ascii'))
                connection.flush()
                print(f'TX: {message}', flush=True)
                sequence += 1
                time.sleep(SEND_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print('\nTX stopped.')
    except serial.SerialException as exc:
        raise SystemExit(f'Could not use serial port {args.port}: {exc}') from exc


if __name__ == '__main__':
    main()
