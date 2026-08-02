#!/usr/bin/env python3
"""Copies the shared field file to device folders; network upload is sync.ps1."""
from pathlib import Path
import shutil
root=Path(__file__).resolve().parent
for role in ('computer','drone','rover'):
    shutil.copy2(root/'field.json',root/role/'field.json')
print('field.json copied to all local device folders')
