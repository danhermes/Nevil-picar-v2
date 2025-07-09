# Debugging Documentation Index

This directory contains comprehensive debugging guides for common issues encountered in the Nevil 2.0 project.

## Available Debugging Guides

### Build Issues
- **[Named Pipe Build Hang Issue](named_pipe_build_hang_issue.md)** - Complete guide for resolving CMake build hangs caused by named pipe files created by development tools

## Quick Reference

### Build Problems
| Issue | Symptoms | Quick Fix |
|-------|----------|-----------|
| Build Hanging | CMake copy operations stuck for 6+ minutes | Remove `.lgd-nfy*` files and rebuild |
| Multiple CMake Processes | Several `cmake -E copy_directory` processes stuck | Kill processes, clean pipes, rebuild |

### Common Commands
```bash
# Find and remove named pipes
find . -name ".lgd-nfy*" -type p -delete

# Kill stuck CMake processes
pkill -f "cmake.*copy_directory"

# Clean and rebuild
rm -rf build/nevil_navigation && colcon build --packages-select nevil_navigation
```

## Contributing to Debugging Docs

When adding new debugging guides:
1. Use descriptive filenames: `issue_type_specific_problem.md`
2. Include symptoms, root cause, and resolution steps
3. Provide prevention strategies
4. Add quick reference commands
5. Update this index

## Document Standards

Each debugging guide should include:
- **Issue Summary**: Brief description and impact
- **Technical Details**: Root cause analysis
- **Symptoms**: How to identify the problem
- **Resolution Steps**: Step-by-step fix instructions
- **Prevention Strategies**: How to avoid recurrence
- **Reference Commands**: Quick command reference

---
**Last Updated**: July 9, 2025  
**Total Guides**: 1