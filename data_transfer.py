import subprocess
import os

def transfer_folder_rsync(local_folder, remote_folder, hostname, username):
    # Expand `~` for the local folder only (sender side)
    local_folder = os.path.expanduser(local_folder)
    
    # Command to run rsync with options to skip host verification and use SSH keys
    command = [
        'rsync', '-avz', '--progress',
        '-e', 'ssh -o StrictHostKeyChecking=no',  # Automatically accept host key
        local_folder,
        f"{username}@{hostname}:{remote_folder}"
    ]
    
    subprocess.run(command, check=True)

# Example usage
transfer_folder_rsync(
    local_folder='~/Documents/eit_data/',  # Expands on sender's side
    remote_folder='~/Documents/eit_data/',  # Will expand on receiver's side
    hostname='ThranePI.local',
    username='ThranePI'  # Username on the Raspberry Pi (receiver)
)