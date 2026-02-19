#!/usr/bin/env python3
"""
Steinhart-Hart Coefficient Calculator for NCP15XH103F03RC NTC Thermistor

Usage:
  1. Look up 3 well-spaced R-T points from the Murata NCP15XH103F03RC datasheet
     (e.g., at -20°C, 25°C, and 70°C or 85°C)
  2. Enter them below in DATA_POINTS
  3. Run: python calc_steinhart_hart.py
  4. Copy the resulting A, B, C coefficients into BqDriver.h

The points should be spread across your operating temperature range.
Wider spread = better accuracy across the full range.
"""

import math

# ============================================================
# 3 data points from Murata NCP15XH103F03RC datasheet
# R-T table "Temperature Characteristics (Center Value)"
# Column: NCPppXH103 (B=3380K, 10kΩ)
# Format: (Temperature_°C, Resistance_Ω)
# ============================================================
DATA_POINTS = [
    (-20.0,  68237.0),   # Cold point  - from datasheet R-T table
    ( 25.0,  10000.0),   # Nominal     - R25 = 10kΩ
    ( 85.0,   1452.0),   # Warm point  - from datasheet R-T table
]

# ============================================================
# Full R-T table from Murata NCP15XH103F03RC datasheet
# Column: NCPppXH103 (B=3380K, 10kΩ)
# All 34 points from -40°C to +125°C in 5°C steps
# Used for verification (NOT used for fitting)
# ============================================================
VERIFY_POINTS = [
    (-40.0, 195652.0),
    (-35.0, 148171.0),
    (-30.0, 113347.0),
    (-25.0,  87559.0),
    # -20°C is a fitting point, skip
    (-15.0,  53650.0),
    (-10.0,  42506.0),
    ( -5.0,  33892.0),
    (  0.0,  27219.0),
    (  5.0,  22021.0),
    ( 10.0,  17926.0),
    ( 15.0,  14674.0),
    ( 20.0,  12081.0),
    # 25°C is a fitting point, skip
    ( 30.0,   8315.0),
    ( 35.0,   6948.0),
    ( 40.0,   5834.0),
    ( 45.0,   4917.0),
    ( 50.0,   4161.0),
    ( 55.0,   3535.0),
    ( 60.0,   3014.0),
    ( 65.0,   2586.0),
    ( 70.0,   2228.0),
    ( 75.0,   1925.0),
    ( 80.0,   1669.0),
    # 85°C is a fitting point, skip
    ( 90.0,   1268.0),
    ( 95.0,   1110.0),
    (100.0,    974.0),
    (105.0,    858.0),
    (110.0,    758.0),
    (115.0,    672.0),
    (120.0,    596.0),
    (125.0,    531.0),
]


def compute_steinhart_hart(points):
    """
    Compute Steinhart-Hart coefficients A, B, C from 3 (T_°C, R_Ω) data points.
    
    Solves the linear system:
      1/T₁ = A + B·ln(R₁) + C·(ln(R₁))³
      1/T₂ = A + B·ln(R₂) + C·(ln(R₂))³
      1/T₃ = A + B·ln(R₃) + C·(ln(R₃))³
    """
    assert len(points) == 3, "Exactly 3 data points required"
    
    # Convert to Kelvin and compute ln(R)
    Y = [1.0 / (t + 273.15) for t, r in points]  # 1/T in Kelvin
    L = [math.log(r) for t, r in points]           # ln(R)
    L3 = [l**3 for l in L]                          # (ln(R))³
    
    # Solve 3x3 linear system using Cramer's rule
    # | 1  L[0]  L3[0] | | A |   | Y[0] |
    # | 1  L[1]  L3[1] | | B | = | Y[1] |
    # | 1  L[2]  L3[2] | | C |   | Y[2] |
    
    def det3(m):
        """Determinant of 3x3 matrix (list of 3 rows, each 3 elements)"""
        return (m[0][0] * (m[1][1]*m[2][2] - m[1][2]*m[2][1])
              - m[0][1] * (m[1][0]*m[2][2] - m[1][2]*m[2][0])
              + m[0][2] * (m[1][0]*m[2][1] - m[1][1]*m[2][0]))
    
    M = [[1, L[0], L3[0]],
         [1, L[1], L3[1]],
         [1, L[2], L3[2]]]
    
    D = det3(M)
    if abs(D) < 1e-30:
        raise ValueError("Singular matrix - check data points (must be distinct)")
    
    # Replace columns with Y vector for Cramer's rule
    Ma = [[Y[0], L[0], L3[0]],
          [Y[1], L[1], L3[1]],
          [Y[2], L[2], L3[2]]]
    
    Mb = [[1, Y[0], L3[0]],
          [1, Y[1], L3[1]],
          [1, Y[2], L3[2]]]
    
    Mc = [[1, L[0], Y[0]],
          [1, L[1], Y[1]],
          [1, L[2], Y[2]]]
    
    A = det3(Ma) / D
    B = det3(Mb) / D
    C = det3(Mc) / D
    
    return A, B, C


def steinhart_hart_temp(R, A, B, C):
    """Calculate temperature in °C from resistance using S-H equation."""
    ln_r = math.log(R)
    inv_T = A + B * ln_r + C * ln_r**3
    return (1.0 / inv_T) - 273.15


def beta_temp(R, R0=10000.0, T0=298.15, beta=3380.0):
    """Calculate temperature in °C using simple Beta equation."""
    inv_T = (1.0 / T0) + (1.0 / beta) * math.log(R / R0)
    return (1.0 / inv_T) - 273.15


def main():
    print("=" * 70)
    print("Steinhart-Hart Coefficient Calculator")
    print("NTC: NCP15XH103F03RC (10kΩ, B=3380)")
    print("=" * 70)
    
    print("\nInput data points:")
    for t, r in DATA_POINTS:
        print(f"  {t:+7.1f}°C  →  {r:>10.0f} Ω")
    
    A, B, C = compute_steinhart_hart(DATA_POINTS)
    
    print(f"\n{'─' * 70}")
    print(f"Steinhart-Hart Coefficients:")
    print(f"  A = {A:.10e}")
    print(f"  B = {B:.10e}")
    print(f"  C = {C:.10e}")
    print(f"{'─' * 70}")
    
    print(f"\nC/C++ defines for BqDriver.h:")
    print(f"  #define SH_A  {A:.10e}f")
    print(f"  #define SH_B  {B:.10e}f")
    print(f"  #define SH_C  {C:.10e}f")
    
    # Verify against input points
    print(f"\n{'─' * 70}")
    print("Verification against input points:")
    print(f"  {'T_real':>8s}  {'R':>10s}  {'T_SH':>8s}  {'T_Beta':>8s}  {'Err_SH':>8s}  {'Err_Beta':>8s}")
    print(f"  {'─'*8}  {'─'*10}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}")
    
    for t_real, r in DATA_POINTS:
        t_sh = steinhart_hart_temp(r, A, B, C)
        t_beta = beta_temp(r)
        print(f"  {t_real:+8.2f}  {r:10.0f}  {t_sh:+8.3f}  {t_beta:+8.3f}  {t_sh-t_real:+8.3f}  {t_beta-t_real:+8.3f}")
    
    # Verify against additional points
    if VERIFY_POINTS:
        print(f"\n{'─' * 70}")
        print("Verification against additional datasheet points:")
        print(f"  {'T_real':>8s}  {'R':>10s}  {'T_SH':>8s}  {'T_Beta':>8s}  {'Err_SH':>8s}  {'Err_Beta':>8s}")
        print(f"  {'─'*8}  {'─'*10}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}")
        
        max_err_sh = 0
        max_err_beta = 0
        for t_real, r in VERIFY_POINTS:
            t_sh = steinhart_hart_temp(r, A, B, C)
            t_beta = beta_temp(r)
            err_sh = t_sh - t_real
            err_beta = t_beta - t_real
            max_err_sh = max(max_err_sh, abs(err_sh))
            max_err_beta = max(max_err_beta, abs(err_beta))
            print(f"  {t_real:+8.2f}  {r:10.0f}  {t_sh:+8.3f}  {t_beta:+8.3f}  {err_sh:+8.3f}  {err_beta:+8.3f}")
        
        print(f"\n  Max |error| Steinhart-Hart: {max_err_sh:.3f}°C")
        print(f"  Max |error| Beta equation:   {max_err_beta:.3f}°C")
    
    print(f"\n{'=' * 70}")
    print("NEXT STEPS:")
    print("  1. Verify DATA_POINTS against YOUR Murata NCP15XH103F03RC datasheet")
    print("  2. Optionally: Use 3 measured R-T points (BME280 reference + back-")
    print("     calculated R_NTC from the TS% reading) for best accuracy")
    print("  3. Copy the A, B, C coefficients into BqDriver.h")
    print("  4. Replace calculateBatteryTemp() Beta equation with S-H equation")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()
