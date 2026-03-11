# boot.py -- run on boot-up

try:
    import main
    main.main()
except Exception as e:
    # If the main application fails, keep REPL available
    print("Error starting main:", e)
