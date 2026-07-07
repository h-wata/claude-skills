---
name: hard-task-playbook
description: >-
  Metacognitive playbook for tackling hard, multi-step tasks: how to decompose
  them, how to verify your own work before declaring it done, and how to decide
  the next action when stuck or between steps. Use this whenever a task is
  large, ambiguous, spans many files or systems, has failed once already, or
  when you notice yourself unsure what to do next — even if the user just says
  "fix this" or "make it work". Especially valuable for debugging sessions,
  refactors, migrations, and anything where a wrong early assumption is
  expensive.
---

# Hard Task Playbook

Written by Fable 5 for Opus 4.8. These are the habits that separate a run that
converges from a run that thrashes. None of them are exotic; the failure mode
is skipping them under time pressure, precisely when they matter most.

## Part 1 — Decomposing hard tasks

### Find the load-bearing question first

Most hard tasks have one uncertainty that dominates all others: an assumption
that, if wrong, invalidates everything built on top of it. Before planning
steps, name that uncertainty explicitly and resolve it first — with a small
experiment, a targeted read of the code, or a question to the user.

Bad decomposition orders steps by convenience ("first set up, then implement,
then test"). Good decomposition orders them by *information gained per unit of
effort*: do the step that could kill the plan earliest, first.

### Decompose along seams, not along size

Splitting a task into equal-sized chunks produces pieces that all depend on
each other. Instead, cut where the interfaces are:

- **Independently verifiable** — each piece has its own definition of done you
  can check without finishing the rest.
- **Independently revertible** — abandoning one piece doesn't unwind the others.
- **Narrow contracts** — pieces communicate through a small, explicit surface
  (a function signature, a file format, a schema), agreed on before work starts.

If you can't state what "done" means for a sub-piece in one sentence, the cut
is in the wrong place.

### Scout before committing

For any task touching unfamiliar code, spend the first fraction of the effort
purely on reconnaissance: where does the relevant logic live, what already
exists that you'd otherwise reinvent, what conventions does this codebase
follow. Write down (in a todo list or scratch note) the 3–5 facts your plan
depends on. A plan built on unverified guesses about the codebase isn't a plan;
it's a bet.

### Keep a live plan, and let reality edit it

Maintain an explicit task list for anything over ~3 steps. But treat it as a
hypothesis: when a step reveals the plan was wrong, stop and re-plan rather
than forcing the remaining steps through. The most expensive failure pattern
is momentum — continuing to execute a plan you already have evidence against.

## Part 2 — Verifying your own work

### Verification means running it, not rereading it

Rereading your own code confirms your own assumptions back to you. Real
verification exercises the change end-to-end: run the test, run the app, feed
it the input that motivated the task, and *observe* the behavior. If a change
has a runtime surface, drive it. "It compiles" and "the diff looks right" are
not evidence.

### Verify the claim you're about to make

Before saying "done", "fixed", or "this works", ask: what evidence do I
actually hold for that sentence? For each claim, there should be an observation
— a passing test you ran this session, output you saw, a behavior you
triggered. If the honest answer is "I inferred it", either go get the
observation or downgrade the claim ("I've made the change; I haven't been able
to run X, so please verify Y").

### Attack your work like a skeptic

After the happy path works, spend a moment deliberately trying to break it:

- The input that's empty, huge, malformed, or concurrent.
- The caller you didn't think about — search for other call sites of anything
  whose behavior you changed.
- The invariant you might have silently broken — what did the old code
  guarantee that the new code no longer does?

One genuine attempt to refute your own work catches more than three rereads.

### Report failures as faithfully as successes

If a test fails, say so and show the output. If you skipped a verification
step, say that. A confident summary that papers over an unverified step costs
the user far more than an honest "3 of 4 pass; the 4th fails on X and here's
why I think so."

## Part 3 — Deciding what to do next

### The next action is the cheapest one that reduces the most uncertainty

Between steps, don't default to "the next item on the list". Ask: what am I
most unsure about right now, and what's the cheapest action that would settle
it? Sometimes that's the next planned step; often it's a 10-second check that
prevents a 10-minute detour.

### When stuck, change what you're varying

Two failed attempts at the same approach is the signal to stop iterating on
the attempt and start questioning the approach. Concretely, escalate through
these levels rather than looping on one:

1. **Same approach, better information** — add logging, isolate a minimal
   reproduction, read the actual error instead of pattern-matching it.
2. **Different approach, same goal** — is there another mechanism entirely?
3. **Question the goal** — is the task as stated the right task? Surface the
   mismatch to the user instead of silently redefining it.

A signal that pattern-matches a known failure may have a different cause;
verify the diagnosis before applying the remembered cure.

### Distinguish "blocked" from "uncomfortable"

Stop and ask the user only when the decision is genuinely theirs: a scope
change, a destructive action, a tradeoff the code can't answer. Everything
else — missing information you can look up, an error you can retry, a detail
you can decide by convention — handle yourself and note the decision in your
summary. Asking permission to do the obvious thing is a stall, not caution.

### Know when to stop

End the turn when either (a) the task is done and verified, or (b) you are
blocked on input only the user can provide. Before ending, check your last
paragraph: if it's a promise ("I'll…", "next I would…"), that's work you should
do now, not narrate. And if you notice the task is done, stop — polishing past
the point of "verified and done" adds risk, not value.

## Quick self-check (use at every milestone)

1. What was the load-bearing assumption, and did I verify it?
2. For each claim in my summary, what did I actually observe?
3. Did I try to break my own change at least once?
4. Is my next action the highest-information one, or just the next one?
5. Am I continuing a plan I already have evidence against?
