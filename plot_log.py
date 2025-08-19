import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

def get_file_index(filepath):
    """Extracts the index number from a log filename (e.g., L100_5.TXT -> 5)."""
    basename = os.path.basename(filepath)
    try:
        # L100_5.TXT -> ['L100', '5.TXT'] -> '5.TXT' -> ['5', 'TXT'] -> '5' -> 5
        index_str = basename.split('_')[1].split('.')[0]
        return int(index_str)
    except (IndexError, ValueError):
        # If the filename is not in the expected format, return a low value
        # so it gets sorted to the end.
        return -1

def find_log_files(directory):
    """
    Finds all .TXT files in the specified directory, sorts them by index number (descending).
    
    Args:
        directory (str): The path to the directory to search.
        
    Returns:
        list: A sorted list of full paths to the .TXT files.
    """
    try:
        files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.TXT')]
        # Sort files by the index in the filename, descending (e.g., L100_1.TXT before L100_0.TXT)
        files.sort(key=get_file_index, reverse=True)
        return files
    except FileNotFoundError:
        print(f"Error: The directory '{directory}' was not found.")
        return []

def select_file(files, action_prompt="analyze"):
    """
    Displays a numbered list of files to the user and prompts for a selection.
    
    Args:
        files (list): A list of file paths.
        action_prompt (str): The action verb to use in the prompt (e.g., "analyze", "delete").
        
    Returns:
        str or None: The path of the selected file, or None if the user chooses to exit.
    """
    print(f"\nPlease select a log file to {action_prompt} (sorted by index, descending):")
    for i, file_path in enumerate(files):
        filename = os.path.basename(file_path)
        print(f"  {i+1}: {filename}")
    
    print("  0: Return to main menu")

    while True:
        try:
            choice = int(input("\nEnter your choice (number): "))
            if 0 <= choice <= len(files):
                if choice == 0:
                    return None
                return files[choice - 1]
            else:
                print("Invalid choice. Please enter a number from the list.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def get_remaining_pid_constants():
    """
    Prompts the user to enter Ki and Kd values.
    
    Returns:
        tuple: A tuple containing (Ki, Kd) as floats.
    """
    print("\nEnter the remaining PID constants for simulation (Ki and Kd).")
    while True:
        try:
            ki = float(input("Enter Ki: "))
            kd = float(input("Enter Kd: "))
            return ki, kd
        except ValueError:
            print("Invalid input. Please enter numeric values for the constants.")

def calculate_pid_values(df, Kp, Ki, Kd):
    """
    Calculates PID values based on the error data from the log file.
    This mimics the logic from the Arduino code.
    
    Args:
        df (pd.DataFrame): DataFrame containing 'Time' and 'Error' columns.
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        
    Returns:
        list: A list of calculated PID values.
    """
    pid_values = []
    I = 0
    erroAnterior = 0
    OFFSET = 0 # Based on constants.h

    for erro in df['Error']:
        P = erro
        I = I + P
        # Constrain the integral term, similar to the Arduino code
        I = max(-255, min(255, I))
        D = erro - erroAnterior
        
        PID = (Kp * P) + (Ki * I) + (Kd * D) + OFFSET
        pid_values.append(PID)
        
        erroAnterior = erro
        
    return pid_values

def plot_data(df, Kp, Ki, Kd):
    """
    Plots the Error vs. Time and PID vs. Time graphs.
    
    Args:
        df (pd.DataFrame): The DataFrame containing all data.
        Kp (float): Proportional gain used.
        Ki (float): Integral gain used.
        Kd (float): Derivative gain used.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6), sharex=True)
    fig.suptitle(f"Analysis of PID Control with Kp={Kp}, Ki={Ki}, Kd={Kd}", fontsize=16)

    # Plot 1: Error vs. Time
    ax1.plot(df['Time'], df['Error'], label='Error', color='blue')
    ax1.set_title('Error vs. Time')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('Error')
    ax1.set_ylim(-6, 6) # Set fixed y-axis for error
    ax1.grid(True)
    ax1.legend()

    # Plot 2: PID vs. Time
    ax2.plot(df['Time'], df['PID'], label='Simulated PID', color='red')
    ax2.set_title('Simulated PID Output vs. Time')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('PID Value')
    # Set dynamic y-axis based on Kp. Handles Kp=0 case.
    pid_limit = max(6 * Kp, 10) # Avoid a flat line if Kp is 0
    ax2.set_ylim(-pid_limit, pid_limit)
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def run_analysis_flow(log_dir):
    """Handles the entire file analysis and plotting workflow."""
    while True:
        log_files = find_log_files(log_dir)
        if not log_files:
            print("\nNo .TXT log files found to analyze.")
            input("Press Enter to return to the main menu...")
            break

        selected_file = select_file(log_files, action_prompt="analyze")
        if selected_file is None:
            break # Return to main menu

        # Extract Kp from filename
        basename = os.path.basename(selected_file)
        kp = None
        try:
            # Extracts the number between 'L' and '_'
            kp_str = basename.split('L')[1].split('_')[0]
            kp = float(kp_str)
            print(f"\nKp value automatically detected from filename: {kp}")
        except (IndexError, ValueError):
            print(f"\nError: Could not automatically detect Kp from filename '{basename}'.")
            print("Please ensure the file is named in the format 'L<Kp>_*.TXT'.")
            print("Restarting file selection...\n")
            continue # Go to the next loop iteration, asking for a file again.

        ki, kd = get_remaining_pid_constants()

        df = pd.read_csv(selected_file)
        df.name = selected_file # Store filename for plotting title
        df['PID'] = calculate_pid_values(df, kp, ki, kd)

        plot_data(df, kp, ki, kd)

def run_deletion_flow(log_dir):
    """Handles the file deletion workflow."""
    while True:
        log_files = find_log_files(log_dir)
        if not log_files:
            print("\nNo .TXT log files found to delete.")
            input("Press Enter to return to the main menu...")
            break

        selected_file = select_file(log_files, action_prompt="delete")
        if selected_file is None:
            break # Return to main menu

        basename = os.path.basename(selected_file)
        try:
            confirm = input(f"Are you sure you want to permanently delete '{basename}'? (y/n): ").lower()
            if confirm == 'y':
                os.remove(selected_file)
                print(f"File '{basename}' has been deleted.")
            else:
                print("Deletion cancelled.")
        except Exception as e:
            print(f"An error occurred while deleting the file: {e}")
        
        print("-" * 20)

def main():
    log_dir = 'E:\\'
    while True:
        print("\n--- Main Menu ---")
        print("(1) - Analyze log file")
        print("(2) - Delete log files")
        print("(0) - Exit")
        
        choice = input("Enter your choice: ")

        if choice == '1':
            run_analysis_flow(log_dir)
        elif choice == '2':
            run_deletion_flow(log_dir)
        elif choice == '0':
            print("Exiting program.")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()
