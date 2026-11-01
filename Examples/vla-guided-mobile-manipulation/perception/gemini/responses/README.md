# Sample responses in the Gemini Robotics-ER format

These files let the example run with no Gemini key and no network request. They are
**not** answers from Gemini: they were generated from the apartment scene as it renders,
and written in the response format Gemini Robotics-ER uses.

With no live key (`cfg.provider.mode = 'replay'`, the default), `hmmProviderRequest`
serves the file whose name contains `hmmReplayTag` of the run's instruction, so the
instruction is what selects the file. Pick one of the sentences below from the app's
example list, or set `cfg.task.instruction` to it. Any other instruction is refused with
`ReplayMiss` and nothing executes; to try your own sentences, use a Gemini key.

| file | instruction that selects it |
|------|-----------------------------|
| `transfer_36117350.json` | Pick up the red mug from the side table and set it down on the kitchen counter. |
| `refine_36117350.json` | the same instruction: the wrist camera's refinement of the grasp point |
| `navigate_9c602578.json` | Drive to the open floor near the mug. |
| `home_8a93e790.json` | go back to the home position |

Two schemas, kept apart by the `refine` file-name prefix: `refine_*.json` answer the wrist
refinement with one `{"found", "point", "confidence"}` object; the others answer the wide
grounding call with an `{"actions": [...]}` array.

Nothing here is a credential and nothing here is machine-specific.
