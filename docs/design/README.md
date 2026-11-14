# Design docs

A design doc describes something before it's built: what problem it solves,
the approach, the alternatives, and the open questions. It is reviewed, then
built, then kept as the record of why the code looks the way it does.

## Status

Each design starts with a status line:

| Status | Meaning |
| --- | --- |
| **Draft** | Being written; not ready for review. |
| **Proposed** | Ready for review. Comments and changes expected. |
| **Accepted** | Agreed; work can start. Open questions may remain, marked as such. |
| **Built** | Implemented. The doc is updated to match what was built, and links to the code. |
| **Superseded** | Replaced by another design (linked). |

When a design settles a lasting choice, it's also added to
[decisions.md](../decisions.md).

## Template

```markdown
# <Title>

Status: Draft · Author: <name> · Updated: <YYYY-MM-DD>

## Summary
Two or three sentences: what and why.

## Goals / Non-goals

## Background
What exists today that this builds on (link to pipeline pages and headers).

## Design
The approach, with data structures and API sketches.

## Accuracy
How results carry uncertainty, and what is reported with them.

## Alternatives considered

## Rollout
Phases, each useful on its own.

## Open questions
```

## Index

| Design | Status |
| --- | --- |
| [Garden model](garden-model.md) | Proposed |
| [Plant knowledge data (vg_data)](plant-data.md) | Proposed |
