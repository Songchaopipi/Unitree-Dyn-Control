"""Small OSQP compatibility helpers."""

from __future__ import annotations


def setup_osqp(prob, **kwargs):
    """Set up OSQP across versions with different polish setting names."""
    try:
        prob.setup(**kwargs)
    except TypeError as exc:
        if "polishing" not in str(exc) or "polishing" not in kwargs:
            raise
        kwargs = dict(kwargs)
        kwargs["polish"] = kwargs.pop("polishing")
        prob.setup(**kwargs)
