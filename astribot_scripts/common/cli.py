"""Shared command-line argument validation."""
import argparse
import math


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('value must be a positive integer')
    return number


def positive_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0:
        raise argparse.ArgumentTypeError('rate must be finite and positive')
    return rate

