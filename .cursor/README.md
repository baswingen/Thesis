# Cursor Workflow Optimization Guide

This guide explains how to optimize your workflow with Cursor features for your Master Thesis project.

## Table of Contents
1. [Cursor Rules](#cursor-rules)
2. [Chat & Composer](#chat--composer)
3. [Codebase Indexing](#codebase-indexing)
4. [Autocomplete & Inline Suggestions](#autocomplete--inline-suggestions)
5. [Codebase Search](#codebase-search)
6. [Keyboard Shortcuts](#keyboard-shortcuts)
7. [Project-Specific Tips](#project-specific-tips)

---

## Cursor Rules

The `.cursor/rules` file contains project-specific guidelines that help Cursor understand your codebase better. The AI assistant will:
- Follow your coding standards and patterns
- Understand your project structure
- Respect data privacy requirements
- Use appropriate error handling patterns

**How to use:**
- The rules are automatically applied when you use Chat or Composer
- You can reference them by saying "follow the project rules" in chat
- Update `.cursor/rules` as your project evolves

---

## Chat & Composer

### Chat (Cmd/Ctrl + L)
Use Chat for:
- **Quick questions**: "How does EMG synchronization work?"
- **Code explanations**: "Explain this signal processing function"
- **Debugging help**: "Why is my IMU connection failing?"
- **Documentation**: "Generate docstring for this function"

**Tips:**
- Reference specific files: "Look at `src/emg_acquisition.py` and explain..."
- Ask for examples: "Show me how to use the EMG preprocessing module"
- Request refactoring: "Refactor this to follow project patterns"

### Composer (Cmd/Ctrl + I)
Use Composer for:
- **Multi-file changes**: "Add error handling to all acquisition modules"
- **Feature implementation**: "Implement MVC normalization for EMG data"
- **Refactoring**: "Update all signal processing functions to use type hints"
- **Documentation**: "Add docstrings to all functions in `src/emg_processing.py`"

**Tips:**
- Be specific about scope: "In the `src/` directory, add..."
- Break down large tasks: "First add the function, then update tests"
- Review changes before accepting (use diff view)

---

## Codebase Indexing

Cursor automatically indexes your codebase to provide better suggestions. For this project:

### What Gets Indexed
- ✅ All Python files in `src/`, `trial/`, `tests/`
- ✅ Configuration files (`setup.py`, `requirements.txt`)
- ✅ README documentation
- ❌ `database/` folder (excluded via `.cursorignore` - contains sensitive participant data)
- ❌ `venv/` and `__pycache__/` (excluded automatically)
- ❌ `tmsi-python-interface/` (large third-party library, may be excluded)

### Improving Indexing
1. **Use `.cursorignore`** to exclude large/unnecessary folders:
   ```
   database/
   venv/
   __pycache__/
   *.pyc
   tmsi-python-interface/Documentation/
   ```

2. **Index status**: Check bottom-right corner for indexing progress
3. **Manual refresh**: Restart Cursor if indexing seems stale

---

## Autocomplete & Inline Suggestions

Cursor provides AI-powered autocomplete as you type.

### Best Practices
- **Type slowly**: Give Cursor time to analyze context
- **Accept suggestions**: Tab to accept, Esc to dismiss
- **Trigger manually**: Cmd/Ctrl + Space for inline suggestions
- **Multi-line**: Cursor can suggest entire function bodies

### For Your Project
- Signal processing functions: Type function name, Cursor suggests parameters
- Hardware initialization: Cursor knows your device setup patterns
- HDF5 operations: Cursor suggests proper data structure patterns
- Error handling: Cursor follows your project's error handling patterns

---

## Codebase Search

### Semantic Search (Cmd/Ctrl + Shift + F)
Search by meaning, not just text:
- "How is EMG data synchronized with IMU?"
- "Where are HDF5 files created?"
- "Find all signal filtering functions"

### Regular Search (Cmd/Ctrl + F)
For exact text matches:
- Function names: `def process_emg`
- Class names: `class EMGAcquisition`
- Error messages: `SerialException`

### Tips
- Use semantic search for understanding code flow
- Use regular search for finding specific implementations
- Combine with Chat: "Search for synchronization code and explain it"

---

## Keyboard Shortcuts

### Essential Shortcuts
- **Cmd/Ctrl + L**: Open Chat
- **Cmd/Ctrl + I**: Open Composer
- **Cmd/Ctrl + K**: Inline edit (select code, then use this)
- **Cmd/Ctrl + Shift + F**: Semantic search
- **Tab**: Accept autocomplete suggestion
- **Esc**: Dismiss suggestion

### Code Actions
- **Cmd/Ctrl + .**: Quick fix / code actions
- **F2**: Rename symbol (works across files)
- **Cmd/Ctrl + Click**: Go to definition

---

## Project-Specific Tips

### 1. Signal Processing Workflow
```
1. Use Chat to understand existing processing pipeline
2. Use Composer to add new processing steps
3. Use inline edit (Cmd+K) to modify filter parameters
4. Use semantic search to find similar processing code
```

### 2. Hardware Integration
```
1. Search for "Arduino connection" to find connection patterns
2. Use Chat: "How do I handle IMU disconnection errors?"
3. Check tests/ for hardware testing examples
4. Use Composer to add error handling to acquisition code
```

### 3. Data Management
```
1. Use semantic search: "How is trial data stored?"
2. Reference README/database/README.md for data format
3. Use Chat to generate HDF5 loading/saving code
4. Use Composer to add new metadata fields
```

### 4. Testing & Debugging
```
1. Use Chat: "Create a test for this function"
2. Use Composer: "Add unit tests for signal preprocessing"
3. Search for "example" in tests/ to see usage patterns
4. Use inline edit to fix bugs quickly
```

### 5. Documentation
```
1. Use Chat: "Generate docstring for this function"
2. Use Composer: "Add docstrings to all functions in this file"
3. Reference existing README files for style
4. Use semantic search to find similar documented functions
```

---

## Advanced Features

### Codebase Chat
- Right-click on a file/folder → "Chat with [file/folder]"
- Ask questions about specific modules
- Get explanations of complex code sections

### Terminal Integration
- Cursor's terminal has AI assistance
- Ask: "Run the test suite" or "Install dependencies"
- Get help with command-line operations

### Git Integration
- Use Chat to write commit messages
- Ask: "What files have changed?"
- Get help with merge conflicts

---

## Troubleshooting

### Cursor Not Understanding Context
- Check that files are indexed (bottom-right status)
- Ensure `.cursorignore` isn't excluding important files
- Try restarting Cursor

### Suggestions Not Appearing
- Check internet connection (some features require online)
- Verify Cursor settings → AI features enabled
- Try manual trigger: Cmd/Ctrl + Space

### Slow Performance
- Exclude large folders in `.cursorignore`
- Reduce scope of Composer changes
- Use Chat for single-file questions instead of Composer

---

## Quick Reference

| Task | Tool | Example |
|------|------|---------|
| Understand code | Chat | "Explain this function" |
| Modify multiple files | Composer | "Add error handling to all acquisition modules" |
| Quick edit | Inline (Cmd+K) | Select code, Cmd+K, describe change |
| Find code | Semantic Search | "Where is EMG filtering implemented?" |
| Generate code | Chat/Composer | "Create a function to normalize EMG data" |
| Fix bugs | Chat + Inline | "Fix this error" then Cmd+K on the fix |
| Add tests | Composer | "Add unit tests for this module" |
| Documentation | Chat | "Generate docstring for this function" |

---

## Next Steps

1. **Try Chat**: Ask "How does the EMG acquisition module work?"
2. **Try Composer**: Request "Add type hints to `src/emg_processing.py`"
3. **Try Inline Edit**: Select some code, press Cmd+K, describe a change
4. **Update Rules**: Customize `.cursor/rules` for your specific needs
5. **Exclude Files**: Update `.cursorignore` if needed

Happy coding! 🚀
