"""Shared setup for the test suite.

One job: collect each test's dead Tk objects on the main thread before the
next test starts.

Every windowed test builds a real Tk window and destroys it, which leaves the
StringVars, widgets and images behind as garbage. Those have __del__ methods
that call back into Tcl, and Python's collector runs on whichever thread
happens to cross the allocation threshold - which, in a suite full of reader
threads, fake serial servers and calibration workers, is usually not the main
one. A Tcl call from the wrong thread does not raise; it panics and aborts the
process, so no except can catch it and the run dies with a core dump partway
through an unrelated file.

Collecting deliberately, on the main thread, between tests means there is
nothing left for a worker thread to trip over. This is why the suite passes
run whole rather than only file by file.
"""

import gc

import pytest


@pytest.fixture(autouse=True)
def _collect_tk_garbage():
    yield
    # After teardown, so the window destroyed by a fixture is included.
    gc.collect()
# def
