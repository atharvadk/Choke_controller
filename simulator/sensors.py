"""
Sensor model: adds noise, delay, quantization, and occasional faults.
"""

import random
from collections import deque
from typing import Dict, Any


def _apply_noise(value: float, std: float) -> float:
    if std <= 0.0:
        return value
    return value + random.gauss(0.0, std)


def _apply_delay(value: float, buffer: deque) -> float:
    """Simple delay line: push new value, pop oldest."""
    buffer.append(value)
    if len(buffer) == 0:
        return value
    return buffer.popleft()


def _apply_quantization(value: float, step: float) -> float:
    if step <= 0.0:
        return value
    return round(value / step) * step


def _apply_fault(value: float, prob: float, ftype: str, fparam: float) -> float:
    if random.random() > prob:
        return value
    if ftype == 'offset':
        return value + fparam
    elif ftype == 'stuck':
        return fparam
    elif ftype == 'spike':
        return value + fparam
    else:
        return value


def measure(
    true_value: float,
    sensor_cfg: Any,
    state: Dict[str, Any],
    var_name: str,
) -> float:
    """
    Apply sensor model to a true value.

    Args:
        true_value: actual physical value.
        sensor_cfg: object with attributes:
            noise_std (float), delay_steps (int), quant_step (float),
            fault_prob (float), fault_type (str), fault_param (float).
        state: dict that persists across calls; used to store delay buffers.
        var_name: identifier for the variable (e.g., 'Pwh') to index its delay buffer.

    Returns:
        Measured value after noise, delay, quantization, faults.
    """
    # Noise
    noisy = _apply_noise(
        true_value,
        getattr(sensor_cfg, 'noise_std', 0.0),
    )

    # Delay
    delay_steps = int(getattr(sensor_cfg, 'delay_steps', 0))
    if delay_steps > 0:
        buf_key = f'{var_name}_delay_buf'
        if buf_key not in state:
            state[buf_key] = deque([0.0] * delay_steps, maxlen=delay_steps)
        else:
            buf = state[buf_key]
            if buf.maxlen != delay_steps:
                # resize buffer preserving newest values
                old_list = list(buf)
                if len(old_list) > delay_steps:
                    old_list = old_list[-delay_steps:]
                else:
                    # pad with zeros at front
                    needed = delay_steps - len(old_list)
                    old_list = [0.0] * needed + old_list
                state[buf_key] = deque([0.0] * delay_steps, maxlen=delay_steps)
                state[buf_key].extend(old_list)
        delayed = _apply_delay(noisy, state[buf_key])
    else:
        delayed = noisy

    # Quantization
    quantized = _apply_quantization(
        delayed,
        getattr(sensor_cfg, 'quant_step', 0.0),
    )

    # Faults
    fault_prob = getattr(sensor_cfg, 'fault_prob', 0.0)
    fault_type = getattr(sensor_cfg, 'fault_type', 'none')
    fault_param = getattr(sensor_cfg, 'fault_param', 0.0)

    measured = _apply_fault(
        quantized,
        fault_prob,
        fault_type,
        fault_param,
    )
    return measured


# Convenience alias
def apply_sensor(value: float, cfg: Any, st: Dict[str, Any], name: str) -> float:
    return measure(value, cfg, st, name)