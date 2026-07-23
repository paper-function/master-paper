# Alpha + bank-reversal down-press test

This campaign keeps positive alpha and commands large bank reversal to turn lift downward. Original ADRC structure is unchanged.

| Case | OK | Stable | Downpress | dH m | avg Hdot m/s | min theta deg | max alpha err | max miu err | max surface | sat frac |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| bank60_alpha8 | 1 | 1 | 0 | -617.5 | -15.4 | -0.44 | 8.00 | 16.42 | 4.1 | 0.00 |
| bank90_alpha8 | 1 | 1 | 1 | -3449.2 | -76.6 | -2.12 | 8.00 | 23.65 | 4.1 | 0.00 |
| bank120_alpha8 | 1 | 1 | 1 | -7803.1 | -156.1 | -4.45 | 8.00 | 29.20 | 4.1 | 0.00 |
| near_inverted_alpha8 | 1 | 1 | 1 | -16313.1 | -296.6 | -9.90 | 8.00 | 35.34 | 4.1 | 0.00 |
| bank90_alpha12 | 1 | 1 | 1 | -2458.0 | -54.6 | -1.77 | 10.00 | 23.64 | 4.1 | 0.00 |
| bank90_lowerQ | 1 | 1 | 1 | -7098.0 | -157.7 | -4.95 | 8.00 | 23.65 | 14.7 | 0.00 |
| bank60_lowerQ | 1 | 1 | 1 | -6454.5 | -143.4 | -4.46 | 8.00 | 10.57 | 14.7 | 0.00 |

Interpretation: if large bank commands fail or produce huge miu error, the limiting factor is bank-reversal authority/tracking, not the alpha channel alone.
