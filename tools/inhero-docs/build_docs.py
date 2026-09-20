"""Entry point for the reviewed Fable v5 conversion."""
import sys
sys.dont_write_bytecode = True
from build_docs_v5 import main

if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
