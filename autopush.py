import os
import datetime
import subprocess
from pathlib import Path

def is_git_repository(path):
    """Check if the given path is a git repository."""
    try:
        result = subprocess.run(
            'git rev-parse --is-inside-work-tree',
            shell=True,
            check=True,
            capture_output=True,
            text=True,
            cwd=path
        )
        return result.stdout.strip() == 'true'
    except subprocess.CalledProcessError:
        return False

def get_commit_count_today():
    try:
        # Get today's date in YYYY-MM-DD format
        today = datetime.datetime.now().strftime('%Y-%m-%d')
        # Get commit count for today
        result = subprocess.run(
            f'git log --after="{today} 00:00:00" --before="{today} 23:59:59" --format="%H" | wc -l',
            shell=True,
            check=True,
            capture_output=True,
            text=True
        )
        return int(result.stdout.strip())
    except subprocess.CalledProcessError as e:
        print(f"Error getting commit count: {e.stderr}")
        return 0

def get_commit_message():
    commit_count = get_commit_count_today()
    default_message = f"Auto-update: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} (Commit #{commit_count + 1} today)"
    user_message = input(f"Enter commit message (press Enter for default: '{default_message}'): ").strip()
    return user_message if user_message else default_message

def get_current_branch():
    try:
        result = subprocess.run(
            'git branch --show-current',
            shell=True,
            check=True,
            capture_output=True,
            text=True
        )
        branch = result.stdout.strip()
        if not branch:
            print("Warning: Could not determine current branch. Are you in a git repository?")
            return None
        return branch
    except subprocess.CalledProcessError as e:
        print(f"Error getting current branch: {e.stderr}")
        return None

def check_git_status():
    """Check if there are any changes to commit."""
    try:
        result = subprocess.run(
            'git status --porcelain',
            shell=True,
            check=True,
            capture_output=True,
            text=True
        )
        return bool(result.stdout.strip())
    except subprocess.CalledProcessError as e:
        print(f"Error checking git status: {e.stderr}")
        return False

def run_git_command(command, cwd=None):
    """Run a git command and return success status."""
    try:
        result = subprocess.run(
            command,
            shell=True,
            check=True,
            capture_output=True,
            text=True,
            cwd=cwd
        )
        if result.stdout.strip():
            print(f"Success: {result.stdout}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {command}")
        print(f"Error message: {e.stderr}")
        return False

def main():
    # Get the directory where the script is being run
    current_dir = Path.cwd()
    
    # Verify we're in a git repository
    if not is_git_repository(current_dir):
        print("Error: Not a git repository. Please run this script from within a git repository.")
        return
    
    print(f"Working in directory: {current_dir}")
    
    # Check if there are any changes to commit
    if not check_git_status():
        print("No changes to commit. Nothing to do.")
        return
    
    # Get current branch
    current_branch = get_current_branch()
    if not current_branch:
        print("Could not determine current branch. Stopping execution.")
        return
    
    print(f"Current branch: {current_branch}")
    
    # Get commit message
    commit_message = get_commit_message()
    
    # Git commands
    commands = [
        'git add .',
        f'git commit -m "{commit_message}"',
        f'git push origin {current_branch}'
    ]
    
    # Execute each command
    for command in commands:
        print(f"\nExecuting: {command}")
        if not run_git_command(command, cwd=current_dir):
            print("Git operation failed. Stopping execution.")
            return
    
    print("\nAll operations completed successfully!")

if __name__ == "__main__":
    main()