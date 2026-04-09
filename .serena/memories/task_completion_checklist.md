# ros_typedb Task Completion Checklist

1. Ensure the `ros_typedb` container with TypeDB server is running.
2. Build the package inside the container:
   - `colcon build --symlink-install --packages-select ros_typedb`
3. Run relevant tests:
   - `scripts/run-tests-docker.sh`
   - or targeted pytest patterns as needed
4. Run mandatory checks:
   - `scripts/run-mandatory-checks-docker.sh`
5. Mention in the final report:
   - commands executed
   - pass/fail summary
   - blockers, if any
