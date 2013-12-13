#!/bin/bash

# Copyright 2010 Ankur Sinha 
# Author: Ankur Sinha <sanjay DOT ankur AT gmail DOT com> 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# File : 
#

sed -n "50 p" Calibrated-HD-synapse-final.txt | tr ' ' '\n'  | sed "/^$/d" > Calibrated-HD-synapse-final-50-row.txt
sed -n "50 p" Calibrated-HD-synapse-1.txt | tr ' ' '\n'  | sed "/^$/d" > Calibrated-HD-synapse-1-50.txt
sed -n "50 p" Calibrated-HD-synapse-2.txt | tr ' ' '\n'  | sed "/^$/d" > Calibrated-HD-synapse-2-50.txt
sed -n "50 p" Calibrated-RotationCellCounterClockwise-synapse.txt | tr ' ' '\n'  | sed "/^$/d" > Calibrated-RotationCellCounterClockwise-synapse-cell50.txt
sed -n "50 p" Calibrated-RotationCellClockwise-synapse.txt | tr ' ' '\n'  | sed "/^$/d" > Calibrated-RotationCellClockwise-synapse-cell50.txt
cut -f 2 0000-Master-debug.txt > 0000-Master-debug-HD.txt; cut -f 4 0000-Master-debug.txt > 0000-Master-debug-FR1.txt; cut -f 5 0000-Master-debug.txt > 0000-Master-debug-FR2.txt
