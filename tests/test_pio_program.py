"""Assemble the firmware with the actual upstream RP2 assembler, count cycles."""

import ast
import types
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


class PIOProgramTests(unittest.TestCase):
    def program(self):
        namespace = {}
        with mock.patch.dict(sys.modules, {
                '_rp2': types.SimpleNamespace(PIO=types.SimpleNamespace(OUT_LOW=2)),
                'micropython': types.SimpleNamespace(const=lambda value: value)}):
            exec((ROOT / 'docs/vendor/rp2-assembler-v1.24.0.py').read_text(), namespace)
        function = next(node for node in ast.parse((ROOT / 'main.py').read_text()).body
                        if isinstance(node, ast.FunctionDef) and node.name == '_dash_pulse_program')
        firmware = {'rp2': types.SimpleNamespace(asm_pio=namespace['asm_pio'],
                                                PIO=types.SimpleNamespace(OUT_LOW=2))}
        exec(compile(ast.Module(body=[function], type_ignores=[]), 'main.py', 'exec'), firmware)
        return firmware['_dash_pulse_program']

    def test_assembled_program_waits_low_and_produces_exact_250000_cycle_high(self):
        program = self.program()
        words = program[0]
        self.assertEqual(len(words), 9)
        self.assertEqual(program[6], 2)  # Output initially low.
        wrap = (program[3] >> 12) & 31
        self.assertEqual(wrap, 8)
        for delay_us in (0, 105000, 150000, 200000):
            fifo = [max(0, delay_us - 5), 249998]
            registers = {1: 0, 2: 0, 7: 0}  # X, Y, OSR.
            pc, cycle, level = 0, 0, 0
            edges = []
            completions = []
            while cycle < 500010:
                word = words[pc]
                opcode = word >> 13
                next_pc = pc + 1 if pc < wrap else 0
                if opcode == 4:
                    if word & 0x80:  # Blocking PULL.
                        if not fifo:
                            break
                        registers[7] = fifo.pop(0)
                    else:  # Nonblocking PUSH completion.
                        self.assertEqual(word, 0x8000)
                        self.assertEqual(level, 0)
                        completions.append(cycle)
                elif opcode == 5:  # MOV register, register.
                    registers[(word >> 5) & 7] = registers[word & 7]
                elif opcode == 0:  # Conditional JMP, post-decrement X or Y.
                    register = {2: 1, 4: 2}[(word >> 5) & 7]
                    if registers[register]:
                        next_pc = word & 31
                    registers[register] = (registers[register] - 1) & 0xffffffff
                elif opcode == 7:  # SET pins.
                    self.assertEqual((word >> 5) & 7, 0)
                    new_level = word & 31
                    if new_level != level:
                        edges.append((cycle, new_level))
                    level = new_level
                else:
                    self.fail('Unexpected PIO opcode')
                cycle += 1
                pc = next_pc
            self.assertEqual(len(edges), 2)
            self.assertEqual(edges[0], (max(5, delay_us), 1))
            self.assertEqual(edges[1][0] - edges[0][0], 250000)
            self.assertEqual(level, 0)
            self.assertEqual(completions, [edges[1][0] + 1])
            self.assertEqual(pc, 0)  # Idle blocking for another request, low.


if __name__ == '__main__':
    unittest.main()
