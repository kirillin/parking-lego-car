#!/usr/bin/env python3
from math import pi
from ev3dev.ev3 import Sound

notes = {'C':0, 'D':2, 'E':4, 'F':5, 'G':7, 'A':9, 'B':11}

jingo_bells =\
    [   ['G4', 1./4], ['E5', 1./4], ['D5', 1./4], ['C5', 1./4],
        ['G4', 2./4], ['--', 2./4], ['G4', 4./4], ['G4', 4./4],
        ['G4', 4./4], ['E5', 4./4], ['D5', 4./4], ['C5', 4./4],
        ['A4', 3./4], ['--', 1./4],
        ['A4', 4./4], ['F5', 4./4], ['E5', 4./4], ['D5', 4./4],
        ['B4', 3./4], ['--', 1./4],
        ['G4', 1./4], ['E5', 1./4], ['D5', 1./4], ['C5', 1./4],
        ['G4', 2./4], ['--', 2./4]
    ]


def getFrequency(note, octave):
    return 16.3516 * 2**((notes[note] + octave * 12.0) / 12.0)


if __name__ == '__main__':
    melody = []
    for note in jingo_bells:
        n = []
        _note = note[0][0]
        if _note in ['-']:
            n.append(0)
            n.append(note[1])
            n.append(0.01)
        else:
            _octave = int(note[0][1])
            n.append(getFrequency(_note, _octave))
            n.append(note[1])
            n.append(0.01)
        melody.append(n)
    Sound.tone(melody).wait()