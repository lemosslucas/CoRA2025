import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
from scipy.signal import find_peaks, savgol_filter

# Mapeia os códigos de desafio para propriedades de plotagem (cor, marcador, etc.)
challenge_map = {
    1: {'label': 'Curva Detectada', 'color': 'gold', 'marker': 'v', 'size': 100},
    2: {'label': 'Faixa de Pedestre', 'color': 'darkviolet', 'marker': 's', 'size': 80},
    3: {'label': 'Área de Parada', 'color': 'red', 'marker': 'X', 'size': 120}
}


def clear_screen():
    """Clears the terminal screen for a cleaner user experience."""
    os.system('cls' if os.name == 'nt' else 'clear')


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
            clear_screen()
            if 0 <= choice <= len(files):
                if choice == 0:
                    return None
                return files[choice - 1]
            else:
                print("Invalid choice. Please enter a number from the list.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def get_pid_constants(default_kp):
    """
    Prompts the user to enter Kp, Ki, and Kd values.
    For Kp, the user can accept a default value by pressing Enter.
    
    Args:
        default_kp (float): The default Kp value (usually from the filename).

    Returns:
        tuple: A tuple containing (Kp, Ki, Kd) as floats.
    """
    print("\nEnter the PID constants for simulation.")
    while True:
        try:
            # Handle Kp - allow empty input to use default
            kp_input = input(f"Enter Kp (default: {default_kp}, press Enter to use): ").strip()
            kp = float(kp_input) if kp_input else default_kp

            # Handle Ki and Kd - require input
            ki = float(input("Enter Ki: "))
            kd = float(input("Enter Kd: "))
            return kp, ki, kd
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
    p_terms = []
    i_terms = []
    d_terms = []

    I = 0
    erroAnterior = 0
    OFFSET = 0 # Based on constants.h

    for erro in df['Error']:
        P = erro
        I = I + P
        # Constrain the integral term, similar to the Arduino code
        I = max(-255, min(255, I))
        D = erro - erroAnterior
        
        p_term_val = Kp * P
        i_term_val = Ki * I
        d_term_val = Kd * D

        PID = p_term_val + i_term_val + d_term_val + OFFSET

        p_terms.append(p_term_val)
        i_terms.append(i_term_val)
        d_terms.append(d_term_val)
        pid_values.append(PID)
        
        erroAnterior = erro
        
    return pd.DataFrame({
        'P_term': p_terms,
        'I_term': i_terms,
        'D_term': d_terms,
        'PID': pid_values
    })

def plot_data(df, Kp, Ki, Kd):
    """
    Plots the Error vs. Time and PID vs. Time graphs.
    
    Args:
        df (pd.DataFrame): The DataFrame containing all data.
    """
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Analysis of PID Control with Kp={Kp}, Ki={Ki}, Kd={Kd}", fontsize=16)

    # Use a Savitzky-Golay filter to smooth the error signal and show the underlying trend.
    # This helps visualize the path the robot *would* take without high-frequency oscillations.
    # window_length must be odd. A larger value means more smoothing.
    # polyorder must be less than window_length. It's the order of the polynomial used to fit the samples.
    try:
        # Ensure window_length is not larger than the data size
        window_length = min(51, len(df['Error']))
        if window_length % 2 == 0: # Make it odd
            window_length -= 1
        
        # The filter requires window_length > polyorder
        if window_length > 3:
             df['Smoothed_Error'] = savgol_filter(df['Error'], window_length=window_length, polyorder=3)
        else:
             df['Smoothed_Error'] = df['Error'] # Not enough data to filter, just copy the data
    except Exception:
        # Fallback if filtering fails for any reason
        df['Smoothed_Error'] = df['Error']

    # Plot 1: Error vs. Time
    ax1.plot(df['Time'], df['Error'],  marker='o', label='Error (Actual Path)', color='blue', alpha=0.4)
    ax1.plot(df['Time'], df['Smoothed_Error'], label='Smoothed Trend', color='cyan', linewidth=2.5)
    ax1.axhline(0, color='k', linestyle='--', linewidth=1, label='Ideal Path (Error=0)')
    ax1.set_title('Error vs Time')
    ax1.set_ylabel('Error')
    ax1.set_ylim(-6, 6) # Set fixed y-axis for error
    ax1.grid(True)
    ax1.legend()

    # Plot 2: PID Component Contributions
    ax2.plot(df['Time'], df['Smoothed_Error'], label='Smoothed Trend', color='green', linewidth=2.5)
    ax2.set_title('Smoothed_Error vs Time')
    ax2.set_ylabel('Error Smoothed')
    ax2.set_ylim(-6, 6) # Set fixed y-axis for error
    ax2.grid(True)
    ax2.legend()

    # Plot 3: Total PID Output vs. Time
    ax3.plot(df['Time'], df['PID'], label='Total PID Output', color='red')
    ax3.set_title('PID Output vs Time')
    ax3.set_xlabel('Time (ms)')
    ax3.set_ylabel('PID Value')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def analyze_performance_and_suggest_pid(df, Kp, Ki, Kd):
    """
    Analyzes the performance based on error data and suggests PID tuning adjustments.
    
    Args:
        df (pd.DataFrame): DataFrame with 'Time' and 'Error' columns.
        Kp (float): The current Proportional gain.
        Ki (float): The current Integral gain.
        Kd (float): The current Derivative gain.
    """
    print("\n--- Análise de Desempenho & Sugestões de Ajuste ---")
    
    error = df['Error'].to_numpy()
    
    # 1. Calcular métricas de desempenho
    mae = np.mean(np.abs(error)) # Erro Médio Absoluto (Mean Absolute Error)
    max_overshoot = np.max(np.abs(error))
    
    print(f"Erro Médio Absoluto (MAE): {mae:.4f}")
    print(f"Máximo Overshoot/Undershoot: {max_overshoot:.4f}")

    # 2. Detectar oscilações usando a busca de picos
    # Encontrar picos (overshoots) e vales (undershoots)
    peaks, _ = find_peaks(error, prominence=0.5) # A proeminência ajuda a filtrar o ruído
    troughs, _ = find_peaks(-error, prominence=0.5)
    num_oscillations = len(peaks) + len(troughs)
    
    print(f"Detectadas {num_oscillations} oscilações significativas (picos/vales).")

    # 3. Fornecer sugestões de ajuste com base em heurísticas
    print("\nSugestões:")
    
    is_oscillating = num_oscillations > 5 # Limiar heurístico para "muita" oscilação
    is_sluggish = mae > 1.0 and not is_oscillating # Heurística para resposta lenta/imprecisa
    has_overshoot = max_overshoot > 2.0 # Heurística para overshoot significativo

    if is_oscillating:
        print("- O sistema parece estar oscilando.")
        print(f"  - Tente reduzir o Kp (atual: {Kp}). Um bom ponto de partida pode ser 50-70% do valor atual.")
        print(f"  - Considere aumentar o Kd (atual: {Kd}) para amortecer as oscilações.")
    elif is_sluggish:
        print("- A resposta parece lenta ou com um grande erro em estado estacionário.")
        print(f"  - Tente aumentar o Kp (atual: {Kp}) para uma resposta mais rápida.")
        print(f"  - Se houver um erro persistente, tente aumentar o Ki (atual: {Ki}) para eliminá-lo.")
    elif has_overshoot:
        print("- O sistema tem um overshoot significativo.")
        print(f"  - Tente aumentar o Kd (atual: {Kd}) para reduzir o overshoot.")
        print(f"  - Alternativamente, você poderia diminuir ligeiramente o Kp (atual: {Kp}).")
    else:
        print("- O desempenho parece razoável. O ajuste fino pode envolver pequenos ajustes em Kp, Ki e Kd.")

    print("\nNota: Estas são sugestões gerais. Os valores ótimos dependem do robô e da pista específicos.")

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

        kp, ki, kd = get_pid_constants(kp)

        df = pd.read_csv(selected_file)
        
        # Calculate PID components and merge them into the main DataFrame
        pid_df = calculate_pid_values(df, kp, ki, kd)
        df = pd.concat([df, pid_df], axis=1)
        
        plot_data(df, kp, ki, kd)

        analyze_performance_and_suggest_pid(df, kp, ki, kd)

def run_deletion_flow(log_dir):
    """Handles the file deletion workflow."""
    while True:
        clear_screen()
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

def change_log_directory(current_dir):
    """
    Prompts the user for a new directory path and validates it.

    Args:
        current_dir (str): The current log directory.

    Returns:
        str: The new, validated directory path, or the original path if input is invalid.
    """
    clear_screen()
    print(f"The current log directory is: {current_dir}")
    new_dir = input("Enter the new log directory path (or press Enter to cancel): ").strip()
    new_dir = new_dir + (":\\" if not new_dir.endswith(":\\") else "")
    
    if not new_dir:
        print("\nOperation cancelled.")
        input("Press Enter to return to the main menu...")
        return current_dir

    if os.path.isdir(new_dir):
        print(f"\nLog directory successfully changed to: {new_dir}")
        input("Press Enter to return to the main menu...")
        return new_dir
    else:
        print(f"\nError: The path '{new_dir}' is not a valid directory.")
        input("Press Enter to return to the main menu...")
        return current_dir

def main():
    log_dir = "C:\\Users\\lucas\\Downloads\\logsSD"

    while True:
        clear_screen()
        print("--- Main Menu ---")
        print(f"Current log directory: {log_dir}")
        print("(1) - Analyze log file")
        print("(2) - Delete log files")
        print("(3) - Change log directory")
        print("(0) - Exit")
        
        choice = input("Enter your choice: ")

        if choice == '1':
            run_analysis_flow(log_dir)
        elif choice == '2':
            run_deletion_flow(log_dir)
        elif choice == '3':
            log_dir = change_log_directory(log_dir)
        elif choice == '0':
            print("Exiting program.")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()
