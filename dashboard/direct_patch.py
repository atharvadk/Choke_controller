#!/usr/bin/env python3
"""
Direct patch of the problematic function in matplotlib.widgets
"""

import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets

# Let's find and patch the exact function causing the issue
# Based on the traceback, it's in a wrapper at line 184 of widgets.py

# Let's see what we can access
print("Available attributes in mwidgets:")
attrs = [attr for attr in dir(mwidgets) if not attr.startswith('_')]
print(attrs[:20])  # First 20

# Let's check if we can find the problematic wrapper
import inspect
import matplotlib.cbook as cbook

# Try to patch the process function in cbook
original_process = cbook.process

def patched_process(func, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except AttributeError as e:
        if "'object' object has no attribute 'inaxes'" in str(e):
            # This is the error we want to ignore
            # But we need to be careful - let's see if we can handle it gracefully
            print(f"Ignoring known matplotlib widget error: {e}")
            return None  # or maybe we should return something else?
        raise

# Apply the patch
cbook.process = patched_process

# Now test
print("Testing with patched cbook.process...")
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)

button_ax = plt.axes([0.7, 0.05, 0.1, 0.075])
button = Button(button_ax, 'Test')

def on_click(event):
    print("Button clicked!")

button.on_clicked(on_click)

print("About to show plot...")
plt.show()
print("Done!")