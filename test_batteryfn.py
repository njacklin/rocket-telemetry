# test battery function
# from https://learn.adafruit.com/adafruit-feather-sense/nrf52-adc

import numpy as np
import matplotlib.pyplot as plt


#
# uint8_t
# mvToPercent(float
# mvolts) {
# if (mvolts < 3300)
# return 0;
#
# if (mvolts < 3600) {
# mvolts -= 3300;
# return mvolts / 30;
# }
#
# mvolts -= 3600;
# return 10 + (mvolts * 0.15F); // thats
# mvolts / 6.66666666
# }

def mvToPercent(mv):
    if mv < 3300:
        return 0

    if mv < 3600:
        return (mv-3300) / 30

    return 10 + ((mv-3600) * 0.15)


# main code
mv = np.linspace(3000,4500,100)
pct = np.array([mvToPercent(mvi) for mvi in mv])

plt.figure(1)

plt.plot(mv, pct)

plt.grid(True)
plt.title("battery percent vs. mv")

plt.ylabel("percent")
plt.xlabel("voltage (mV)")

plt.show()
