"""SoccerSoftware package.

This repo historically treated `SoccerSoftware/` as a plain script folder and
relied on `sys.path` / current working directory.

To make commands like `python -m SoccerSoftware.offence` work reliably (and to
support IDE tooling), we also expose it as a proper Python package.
"""
