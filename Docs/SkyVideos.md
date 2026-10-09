# Sky videos for the SunDial clue screen

The Sprite DV-S1 beside the sundial shows Red Beard's ship at sunset. Each
sundial question puts its clue in the clouds. Driven by `Code/SunDialSky`
(ESP32-S3) from the retained topic `MermaidsTale/SunDial/Clue`.

Reference clip (2026-08-08): 1280x720, 24 fps, 8 s, h264. Sunset, galleon
silhouette with black skull flags, sun sitting on the horizon behind the bow,
purple-orange cumulus across the top third, calm sea, locked-off camera.

## File plan on the SD card

Two files per clue, because the player LOOPS the file it holds and the
"old words fade out, new words fade in" moment must only play once.

| File | Loops? | Content |
|---|---|---|
| 000 | loop | plain sky, no words (idle, before the game and after reset) |
| 001..005 | loop | sky with the words for question 1..5, steady |
| 006 | loop | sky with the "solved" words, steady |
| 011..015 | play once | transition INTO question 1..5: previous words fade out, new words fade in |
| 016 | play once | transition into "solved" |

Firmware sequence on clue N: `0xFC N` (hold file = 00N) then `0x(10+N)`
(play transition 01N once). When the transition ends the player falls back
into the 00N loop by itself, so the cut is on identical frames (see below).
Clue 0 (reset) is just `0xFC 0`, a hard cut back to the plain sky.

## Making the clips so every cut is invisible

1. **Generate ONE base clip B, no text** (prompt below). 8 s is enough.
2. **Hold file 00N** = B played backwards, then B forwards, with the words
   on top the whole time (16 s). Backwards-then-forwards loops seamlessly:
   the last frame equals the first frame of the next loop. Sunset clouds
   look fine in reverse.
3. **Transition file 01N** = B forwards (8 s) with:
   - 0.0-1.0 s previous words steady (for 011 there are no previous words)
   - 1.0-3.0 s previous words fade OUT
   - 3.5-5.5 s new words fade IN
   - 5.5-8.0 s new words steady
   Its last frame is B's last frame, and the hold file STARTS on B's last
   frame (it begins with B reversed), so the player's fall-back into the
   loop lands on the same picture.
4. Export every file with identical settings: 1280x720, 24 fps, h264 high,
   ~12 Mbit/s, AAC audio (silent track is fine). Name them 000.mp4 etc.

## Text look

- Font: a rough, hand-cut serif or brush face (e.g. Pirata One, IM Fell,
  Cinzel Rough). Size: 60-72 px at 720p, letter-spacing +5.
- Colour: pull from the clouds themselves (pale peach #F4C89A to #FFE6C7),
  opacity 85-90 %, then blend mode SCREEN or SOFT LIGHT over the cloud band
  so the letters read as lit cloud, not a subtitle.
- Softness: gaussian blur 2-3 px on the text, plus a copy of the text 20 px
  blur at 40 % underneath as glow. Optional: a slow 0.5 px drift up-left
  over the hold so the words feel like they are part of the sky.
- Position: inside the cloud band, upper third, right of the masts so the
  rigging never crosses the letters. Two lines max.
- Fades: 2 s linear opacity. Nothing else animates.

## Words (2026-10-09 "statement" set, owner's pick; matches SpinStop v3.3.0+ STEPS; five lines each, upper-left cloud band, 34 px)

The riddles never tell players to count. Only the fifth line hints that the
answer has company (or, for the trident, none); Evalee's hint ladder carries
the counting. Build kit (build.sh, base.mp4, scr.ttf) and the four rejected
wordings sit in `Downloads/SunDial_Clue_Videos/draft_*`; the chosen set is
`draft_cryptic`.

| Question | Clue in the clouds |
|---|---|
| 1 lighthouse -> bottle (3) | One speaks in light. I speak in ink. A neck but no head. A mouth but no voice. I did not wash up alone. |
| 2 crab+seahorse -> turtle (7) | Born beneath the sand. Raised beneath the waves. I never leave home, yet I am never home. I did not hatch alone. |
| 3 lighthouse+anchor -> coconut (9) | Three eyes, yet blind. A beard, yet no face. I wait up high, then fall without warning. I do not wait alone. |
| 4 shark+crab -> skull (4) | I once held every secret a pirate knew. Now I am empty, and I cannot stop grinning. I do not grin alone. |
| 5 shark+seahorse -> trident (1) | Three teeth, but I have never tasted a thing. The one who holds me rules all that swim. I share my throne with none. |
| solved | The sun has spoken |

## Prompts

### Base clip (the one you actually need)

Use this in the same generator that made the 2026-08-08 clip. Image-to-video
from a still of that clip keeps the ship identical; text-to-video below if
you start fresh.

> Cinematic wide shot at sea at sunset. A large wooden pirate galleon in
> silhouette, three masts, full sails, black skull-and-crossbones flags,
> sits centre-frame on a calm ocean. The sun rests on the horizon directly
> behind the bow, a glowing orange disc. Towering purple and orange cumulus
> clouds fill the upper third of the frame, lit from below. Gentle swell,
> soft golden reflections on the water. Locked-off camera, no camera move,
> no zoom. Slow, subtle motion only: clouds drifting, water rippling, flags
> stirring. No people, no birds, no text, no letters, no logos, no
> watermark. Photoreal, warm, painterly light. 8 seconds, 24 fps, 16:9.

Negative / avoid (if the tool has the field): text, letters, captions,
subtitles, watermark, camera movement, zoom, fast motion, extra ships,
birds, people, lens flare streaks.

### If you insist on the generator writing the words

Most tools will misspell or morph the letters; expect several tries per
clue and check every frame. Use image-to-video from the base clip's first
frame and add:

> ...Halfway through, the words "COUNT WHAT THE SEA WASHED UP" appear
> written in the clouds as soft, glowing cloud lettering in the upper right,
> fading in gently over two seconds and then holding perfectly still and
> legible. The letters are made of lit cloud, the same peach colour as the
> sky. Exact spelling, all capitals, no other text.

For the "previous words fade away" transition the generator would have to
start with words already there; that is where these tools fall apart, so
do the transitions in the editor.
