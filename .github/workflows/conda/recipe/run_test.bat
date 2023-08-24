colcon test --event-handlers console_direct+ --return-code-on-test-failure

if %errorlevel% neq 0 exit /b %errorlevel%