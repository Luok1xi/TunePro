# Third-Party Notices

This project (`TunePro`, licensed under GNU GPL-3.0-or-later) incorporates adapted
methodologies, safety-guardrail patterns, and Agent Skill workflows derived from
the following open-source project:

## llm-pid-tuner

- **Repository**: https://github.com/KINGSTON-115/llm-pid-tuner
- **License**: MIT License
- **Copyright**: Copyright (c) 2026 Charles Liang
- **Adapted components**:
  - Bounded PID proposal verification, regression rollback logic, robust LLM JSON extraction, and fallback heuristic design in `ai_tuner.py`
  - Standalone zero-dependency Agent Skill structure, evidence-based experiment discipline, and offline telemetry/guardrail CLI patterns in `skills/tc264-pid-tuner/`

```text
MIT License

Copyright (c) 2026 Charles Liang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
