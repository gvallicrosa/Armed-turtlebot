#!/bin/bash
pyrcc4 -o interface_rc.py interface.qrc
pyuic4 -o ui_interface.py interface.ui
