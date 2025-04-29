#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import datetime
class TimeConverter:
    SECONDS_PER_DAY = 86400

    @staticmethod
    def decimal_year_from_date(date):
        """Calculate the decimal year (2022.5) from a 'datetime.date' or 'datetime.datetime'.
        date = datetime.date(2022, 7, 1)
        Check if date is datetime.datetime: Since date is already a 
        datetime.date object, we don't need to convert it.
        year = date.year  # year = 2022
        current_year = datetime.date(2023, 1, 1)  # January 1, 2023
        following_year = datetime.date(2022, 1, 1)  # January 1, 2022
        days_in_year = (current_year - following_year).days  # 365 days
        days_passed = (date - datetime.date(2022, 1, 1)).days  # 181 days
        decimal_year = 2022 + float(181) / 365  # 2022 + 0.4959 ≈ 2022.4959
        """
        if isinstance(date, datetime.datetime):
            date = date.date()
        year = date.year
        current_year = datetime.date(year + 1, 1, 1)
        following_year = datetime.date(year, 1, 1)
        days_in_year = (current_year - following_year).days
        days_passed = (date - datetime.date(year, 1, 1)).days
        return year + float(days_passed) / days_in_year
