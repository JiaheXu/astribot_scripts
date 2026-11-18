"""Shared command-line argument validation."""
import argparse
import math


def positive_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0:
        raise argparse.ArgumentTypeError('rate must be finite and positive')
    return rate

