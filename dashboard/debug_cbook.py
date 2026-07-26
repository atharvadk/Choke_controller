#!/usr/bin/env python3
"""
Debug what's in matplotlib.cbook
"""

import matplotlib.cbook as cbook

print("Attributes in matplotlib.cbook:")
attrs = [attr for attr in dir(cbook) if not attr.startswith('_')]
for attr in sorted(attrs):
    print(f"  {attr}")

# Let's look for callback-related stuff
print("\nLooking for callback-related attributes:")
callback_attrs = [attr for attr in dir(cbook) if 'call' in attr.lower()]
for attr in callback_attrs:
    print(f"  {attr}")

# Let's see what the error traceback mentioned
# It said: File ".../matplotlib/cbook.py", line 390, in process
# So there IS a process function, but maybe it's not directly accessible