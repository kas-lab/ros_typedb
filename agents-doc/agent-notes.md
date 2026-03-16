# Agent Notes

Short checklist for day-to-day feature work.

## Quick start

Start container (if needed):

```bash
docker run -d --rm --name ros_typedb -v /etc/localtime:/etc/localtime:ro -v $PWD:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb sudo typedb server
```

Run tests:

```bash
scripts/run-tests-docker.sh
```

Run mandatory checks:

```bash
scripts/run-mandatory-checks-docker.sh
```

## Before finishing a task

- Confirm relevant tests pass.
- Confirm `scripts/run-mandatory-checks-docker.sh` passes.
- Confirm TypeDB server is running before executing build/test/launch/query commands.
- In final response, include:
  - commands executed
  - pass/fail result
  - blocker (if anything could not run)

## Common failure checks

- Is `ros_typedb` running?
- Was ROS/workspace sourced inside container?
- Is TypeDB reachable at `localhost:1729`?
