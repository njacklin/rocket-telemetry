# least-squares fit for pressure-to-altitude conversion
#
# The equation given at https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html,
# (inversed) is roughly A = ( 44330.0 * (1.0 - pow(pressurePa / offsetPa, 0.1903)) ) .
# This script will take in true A_AGL, P_Pa, and a ground pressure reading P_Ground_Pa,
# for several test points, and then optimize params for the model
# A = c1 * (1 - P/Po)^c2 which can be linearized by taking the log of both sides as
# logA = log(c1) + c2 * log(1-P/Po), and defining c1_ = log(c1).
#
# actually, this is a dumb idea.
# there is not model uncertainty.  There is "altimeter setting" i.e. "P0" uncertainty.
