#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math 
from typing import  Tuple, Union
import os
import datetime
import numpy as np
from ament_index_python.packages import get_package_share_directory


WMM_MODEL_2025_LOWER = 2025.0
WMM_MODEL_2025_UPPER = 2030.0

WMM_SIZE_STANDARD = 12


class GeoMagResult:
    """The Magnetic Components values from ``GeoMag.calculate()``."""

    def __init__(self, time: float, alt: float, glat: float, glon: float) -> None:
        """Time (in decimal year)."""
        self.time: float = time
        """Altitude, -1 to 850km referenced to the WGS 84 ellipsoid OR the Mean Sea Level (MSL)."""
        self.alt: float = alt
        """Geodetic Latitude, -90.00 to +90.00 degrees (North positive, South negative)."""
        self.glat: float = glat
        """Geodetic Longitude, -180.00 to +180.00 degrees (East positive, West negative)."""
        self.glon: float = glon
        """North Component."""
        self.x: float = None
        """East Component."""
        self.y: float = None
        """Vertical Component."""
        self.z: float = None
        """Horizontal Intensity."""
        self.h: float = None
        """Total Intensity."""
        self.f: float = None
        """Geomagnetic Inclination."""
        self.i: float = None
        """Geomagnetic Declination (Magnetic Variation)."""
        self.d: float = None
        """Magnetic grid variation if the current geodetic position is in the arctic or antarctic."""
        self.gv: float = None

    def calculate(self) -> None:
        """Calculate extra result values."""
        # COMPUTE X, Y, Z, AND H COMPONENTS OF THE MAGNETIC FIELD
        cos_d = math.cos(math.radians(self.d))
        sin_d = math.sin(math.radians(self.d))
        cos_i = math.cos(math.radians(self.i))
        sin_i = math.sin(math.radians(self.i))
        
        """
        Represents the northward component of the magnetic field
        It is the projection of the magnetic field vector onto the north-south axis.
        """
        self.x = self.f * (cos_d * cos_i)
        """
        Represents the eastward component of the magnetic field.
        It is the projection of the magnetic field vector onto the east-west axis.
        """
        self.y = self.f * (cos_i * sin_d)
        """
        Represents the vertical component of the magnetic field.
        It is the projection of the magnetic field vector onto the vertical axis (up or down).
        """
        self.z = self.f * sin_i
        """
        Represents the horizontal strength of the magnetic field.
        It is the projection of the magnetic field vector onto the horizontal plane, 
        combining both the X and Y components.
        """
        self.h = self.f * cos_i

class GeoMag:
    def __init__(self,
        coefficients_file: str = None,
        coefficients_data: Tuple = None,
        base_year: Union[str, datetime.datetime] = None,
        high_resolution: bool = False,
    ) -> None:
        self._base_year = base_year
        self._coefficients_data = coefficients_data
        self._coefficients_file = coefficients_file
        self._maxord =  WMM_SIZE_STANDARD
        self._size = self._maxord + 1
        
        # Initialize these lazily
        self._epoch = None
        self._model = None
        self._release_date = None
        self._c = None
        self._cd = None
        self._p = None
        self._fn = None
        self._fm = None
        self._k = None
        
        # Cache for optimization
        self._cache = {
            'last_coords': None,
            'last_time': None,
            'matrices': {}
        }

    @classmethod
    def _get_coefficients_year(cls, year: Union[str, datetime.datetime]) -> str:
        year_value = getattr(year, "year", year)

        if 2025 <= year_value < 2030:
            return "2025"
        else:
            raise ValueError(f"There are no coefficients for the year {year_value}")
      
    def _get_model_filename(self) -> str:
        """Determine the model filename to load the coefficients from."""
    	# Check in the package share directory first
        package_share_dir = get_package_share_directory('true_north_calculator')
        default_file = "WMM.COF"
        share_path = os.path.join(package_share_dir, default_file)
        if os.path.exists(share_path):
              return share_path

        # If not found in share directory, check other locations
        filepath = os.path.dirname(__file__)
        default_path = os.path.join(filepath, default_file)
        cwd_path = os.path.join(os.getcwd(), default_file)

        if os.path.exists(cwd_path):
              return cwd_path

        if os.path.exists(default_path):
              return default_path

        # Try looking in a 'wmm' subdirectory as fallback
        fallback_path = os.path.join(filepath, "wmm", default_file)
        if os.path.exists(fallback_path):
              return fallback_path

        # If we can't find the file anywhere, raise a clear error
        raise FileNotFoundError(
            f"Default coefficient file not found at {share_path}, {default_path}, {cwd_path}, "
            f"or in wmm/ subdirectory. Please specify the correct file path when initializing GeoMag."
    	)
    
         

    def _load_coefficients(self) -> None:
        """Load the coefficients model using NumPy for memory efficiency."""
        if self._epoch is not None:
            return

    
        # Using NumPy arrays for better memory efficiency
        c = np.zeros((self._size, self._size), dtype=float)
        cd = np.zeros((self._size, self._size), dtype=float)
        snorm = np.zeros(self._size**2, dtype=float)
        fn = np.zeros(self._size, dtype=float)
        fm = np.zeros(self._size, dtype=float)
        k = np.zeros((self._size, self._size), dtype=float)

        if self._coefficients_data:
            (epoch, model, release_date), coefficients = self._coefficients_data
        else:
            (epoch, model, release_date), coefficients = self._read_coefficients_data_from_file()

        # READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS
        for n, m, gnm, hnm, dgnm, dhnm in coefficients:
            if m > self._maxord:
                break
            if m > n or m < 0:
                raise ValueError("Corrupt record in model file")
            if m <= n:
                c[m, n] = gnm
                cd[m, n] = dgnm
                if m != 0:
                    c[n, m - 1] = hnm
                    cd[n, m - 1] = dhnm

        # CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED
        snorm[0] = 1.0
        fm[0] = 0.0
        for n in range(1, self._maxord + 1):
            snorm[n] = snorm[n - 1] * float(2 * n - 1) / float(n)
            j = 2
            m = 0
            D1 = 1
            D2 = (n - m + D1) / D1
            while D2 > 0:
                k[m, n] = float(((n - 1) * (n - 1)) - (m * m)) / float(
                    (2 * n - 1) * (2 * n - 3)
                )
                if m > 0:
                    flnmj = float((n - m + 1) * j) / float(n + m)
                    snorm[n + m * self._size] = snorm[n + (m - 1) * self._size] * math.sqrt(flnmj)
                    j = 1
                    c[n, m - 1] = snorm[n + m * self._size] * c[n, m - 1]
                    cd[n, m - 1] = snorm[n + m * self._size] * cd[n, m - 1]
                c[m, n] = snorm[n + m * self._size] * c[m, n]
                cd[m, n] = snorm[n + m * self._size] * cd[m, n]
                D2 -= 1
                m += D1
            fn[n] = float(n + 1)
            fm[n] = float(n)
        k[1, 1] = 0.0

        self._epoch = epoch
        self._model = model
        self._release_date = release_date
        self._c = c
        self._cd = cd
        self._p = snorm
        self._fn = fn
        self._fm = fm
        self._k = k

    def _read_coefficients_data_from_file(self) -> Tuple[Tuple[str, str, str], list]:
        """Read coefficients data from file to be processed by ``_load_coefficients``."""
        data = []
        model_filename = self._get_model_filename()

        with open(model_filename, 'r') as coefficients_file:
            line_data = coefficients_file.readline()
            line_values = line_data.split()
            if len(line_values) != 3:
                raise ValueError("Invalid header in model file")
            epoch, model, release_date = (
                t(s) for t, s in zip((float, str, str), line_values)
            )

            while True:
                line_data = coefficients_file.readline()
                if line_data[:4] == "9999":
                    break
                    
                line_values = line_data.split()
                if len(line_values) != 6:
                    raise ValueError("Corrupt record in model file")
                n, m, gnm, hnm, dgnm, dhnm = (
                    t(s) for t, s in zip((int, int, float, float, float, float), line_values)
                )
                data.append((n, m, gnm, hnm, dgnm, dhnm))

        return (epoch, model, release_date), data

    def calculate(
    self,
    glat: float,
    glon: float,
    alt: float,
    time: float,
    allow_date_outside_lifespan: bool = False,
) -> GeoMagResult:
        """Calculate the Magnetic Components without using caching."""
        # Using NumPy arrays for better memory efficiency
        tc = np.zeros((self._size, self._size), dtype=float)
        dp = np.zeros((self._size, self._size), dtype=float)
        sp = np.zeros(self._size, dtype=float)
        cp = np.zeros(self._size, dtype=float)
        pp = np.zeros(self._size, dtype=float)

        # INITIALIZE CONSTANTS
        sp[0] = 0.0
        cp[0] = pp[0] = 1.0
        dp[0, 0] = 0.0
    
        # Earth's dimensions
        a = 6378.137
        b = 6356.7523142
        re = 6371.2
        a2 = a * a
        b2 = b * b
        c2 = a2 - b2
        a4 = a2 * a2
        b4 = b2 * b2
        c4 = a4 - b4

        self._load_coefficients()

        dt = time - self._epoch
        if (dt < 0.0 or dt > 5.0) and not allow_date_outside_lifespan:
            raise ValueError("Time extends beyond WMM 2025 model 5-year life span")

        rlon = math.radians(glon)
        rlat = math.radians(glat)
        srlon = math.sin(rlon)
        srlat = math.sin(rlat)
        crlon = math.cos(rlon)
        crlat = math.cos(rlat)
        srlat2 = srlat * srlat
        crlat2 = crlat * crlat
        sp[1] = srlon
        cp[1] = crlon

        # CONVERT FROM GEODETIC COORDINATES TO SPHERICAL COORDINATES
        q = math.sqrt(a2 - c2 * srlat2)
        q1 = alt * q
        q2 = ((q1 + a2) / (q1 + b2)) * ((q1 + a2) / (q1 + b2))
        ct = srlat / math.sqrt(q2 * crlat2 + srlat2)
        st = math.sqrt(1.0 - (ct * ct))
        r2 = (alt * alt) + 2.0 * q1 + (a4 - c4 * srlat2) / (q * q)
        r = math.sqrt(r2)
        d = math.sqrt(a2 * crlat2 + b2 * srlat2)
        ca = (alt + d) / r
        sa = c2 * crlat * srlat / (r * d)

        # Compute spherical harmonics
        for m in range(2, self._maxord + 1):
            sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1]
            cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1]

        aor = re / r
        ar = aor * aor
        br = bt = bp = bpp = 0.0
    
        for n in range(1, self._maxord + 1):
            ar = ar * aor
            m = 0
            D3 = 1
            D4 = (n + m + D3) / D3
        
            while D4 > 0:
                # COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS AND DERIVATIVES
                if n == m:
                    self._p[n + m * self._size] = st * self._p[n - 1 + (m - 1) * self._size]
                    dp[m, n] = st * dp[m - 1, n - 1] + ct * self._p[n - 1 + (m - 1) * self._size]
                elif n == 1 and m == 0:
                    self._p[n + m * self._size] = ct * self._p[n - 1 + m * self._size]
                    dp[m, n] = ct * dp[m, n - 1] - st * self._p[n - 1 + m * self._size]
                elif n > 1 and n != m:
                    if m > n - 2:
                        self._p[n - 2 + m * self._size] = 0.0
                    if m > n - 2:
                        dp[m, n - 2] = 0.0
                    self._p[n + m * self._size] = ct * self._p[n - 1 + m * self._size] - self._k[m, n] * self._p[n - 2 + m * self._size]
                    dp[m, n] = ct * dp[m, n - 1] - st * self._p[n - 1 + m * self._size] - self._k[m, n] * dp[m, n - 2]

                # TIME ADJUST THE GAUSS COEFFICIENTS
                tc[m, n] = self._c[m, n] + dt * self._cd[m, n]
                if m != 0:
                    tc[n, m - 1] = self._c[n, m - 1] + dt * self._cd[n, m - 1]

                # ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
                par = ar * self._p[n + m * self._size]
                if m == 0:
                    temp1 = tc[m, n] * cp[m]
                    temp2 = tc[m, n] * sp[m]
                else:
                    temp1 = tc[m, n] * cp[m] + tc[n, m - 1] * sp[m]
                    temp2 = tc[m, n] * sp[m] - tc[n, m - 1] * cp[m]
                bt = bt - ar * temp1 * dp[m, n]
                bp += self._fm[m] * temp2 * par
                br += self._fn[n] * temp1 * par

                # SPECIAL CASE: NORTH/SOUTH GEOGRAPHIC POLES
                if st == 0.0 and m == 1:
                    if n == 1:
                        pp[n] = pp[n - 1]
                    else:
                        pp[n] = ct * pp[n - 1] - self._k[m, n] * pp[n - 2]
                    parp = ar * pp[n]
                    bpp += self._fm[m] * temp2 * parp

                D4 -= 1
                m += D3

        if st == 0.0:
            bp = bpp
        else:
            bp /= st

        # ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO GEODETIC COORDINATES
        bx = -bt * ca - br * sa
        by = bp
        bz = bt * sa - br * ca

        result = GeoMagResult(time, alt, glat, glon)

        # COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND TOTAL INTENSITY (TI)
        bh = math.sqrt((bx * bx) + (by * by))
        result.f = math.sqrt((bh * bh) + (bz * bz))
        result.d = math.atan2(by, bx)
        result.i = math.atan2(bz, bh)

        # COMPUTE MAGNETIC GRID VARIATION
        result.gv = -999.0
        if math.fabs(glat) >= 55.0:
            if glat > 0.0 and glon >= 0.0:
                result.gv = result.d - glon
            if glat > 0.0 and glon < 0.0:
                result.gv = result.d + math.fabs(glon)
            if glat < 0.0 and glon >= 0.0:
                result.gv = result.d + glon
            if glat < 0.0 and glon < 0.0:
                result.gv = result.d - math.fabs(glon)
            if result.gv > +180.0:
                result.gv -= 360.0
            if result.gv < -180.0:
                result.gv += 360.0
        if result.gv == -999.0:
            result.gv = None

        # Calculate extra values
        result.calculate()
    
        return result
