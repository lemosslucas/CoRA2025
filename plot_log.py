import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
from scipy.signal import find_peaks, savgol_filter

# Mapeia os códigos de desafio para propriedades de plotagem (cor, marcador, etc.)
challenge_map = {
    4: {'label': 'Curva Esquerda', 'color': 'gold', 'marker': 'v', 'size': 100},
    2: {'label': 'Faixa de Pedestre', 'color': 'darkviolet', 'marker': 's', 'size': 80},
    3: {'label': 'Área de Parada', 'color': 'red', 'marker': 'X', 'size': 120},
    1: {'label': 'Curva Direita', 'color': 'pink', 'marker': 'v', 'size': 100},
    5: {'label': 'Recuperando a linha', 'color': 'red', 'marker': 'x', 'size': 100},
    6: {'label': 'Curva Duvida', 'color': 'gold', 'marker': 'v', 'size': 100},
    7: {'label': 'Ré', 'color': 'pink', 'marker': 'x', 'size': 100},
    8: {'label': 'Rotatoria', 'color': 'pink', 'marker': 'v', 'size': 100},
    9: {'label': 'analisando contorno', 'color': 'silver', 'marker': '^', 'size': 100},
    10: {'label': 'linha_invertida', 'color': 'orange', 'marker': 's', 'size': 80},
    11: {'label': 'Inversão finalizada', 'color': 'green', 'marker': 'o', 'size': 80},
    12: {'label': 'reajuste faixa', 'color': 'red', 'marker': 'o', 'size': 40},
}


def clear_screen():
    """Clears the terminal screen for a cleaner user experience."""
    os.system('cls' if os.name == 'nt' else 'clear')


def get_sort_params(filepath):
    """
    Extracts Kp and index from a log filename (e.g., L90_1.TXT -> (90.0, 1)).
    This is used as a key for sorting files.
    """
    basename = os.path.basename(filepath)
    try:
        # L90_1.TXT -> ['L90', '1.TXT']
        parts = basename.split('_')
        # 'L90' -> '90'
        kp_str = parts[0].replace('L', '')
        # '1.TXT' -> '1'
        index_str = parts[1].split('.')[0]
        kp = float(kp_str)
        index = int(index_str)
        return (kp, index)
    except (IndexError, ValueError):
        # For malformed filenames, return values that will sort them at the end.
        return (float('inf'), -1)

def find_log_files(directory):
    """
    Finds all .TXT files in the specified directory, sorts them by Kp (ascending)
    and then by index number (descending).
    
    Args:
        directory (str): The path to the directory to search.
        
    Returns:
        list: A sorted list of full paths to the .TXT files.
    """
    try:
        files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.TXT')]
        # Sort by Kp ascending, then by index descending.
        # To sort index descending, we sort by its negative value.
        files.sort(key=lambda f: (get_sort_params(f)[0], -get_sort_params(f)[1]))
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
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f"Analysis of PID Control with Kp={Kp}, Ki={Ki}, Kd={Kd}", fontsize=16)

    # Use a Savitzky-Golay filter to smooth the error signal and show the underlying trend.
    # This helps visualize the path the robot would take without high-frequency oscillations.
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
    ax1.plot(df['Time'], df['Smoothed_Error'], label='Smoothed Trend', color='green', linewidth=2.5)
    ax1.axhline(0, color='k', linestyle='--', linewidth=1, label='Ideal Path (Error=0)')

    # Adiciona marcadores para desafios se a coluna 'Challenge' existir e tiver dados
    if 'Challenge' in df.columns and not df[df['Challenge'] != 0].empty:
        challenges = df[df['Challenge'] != 0]
        plotted_labels = set()

        for _, row in challenges.iterrows():
            challenge_code = int(row['Challenge'])
            if challenge_code in challenge_map:
                props = challenge_map[challenge_code]
                label = props['label']
                
                # Adiciona o rótulo à legenda apenas uma vez por tipo de desafio
                current_label = label if label not in plotted_labels else ""
                
                ax1.scatter(row['Time'], row['Error'],
                            label=current_label,
                            color=props['color'],
                            marker=props['marker'],
                            s=props['size'],
                            zorder=5) # Garante que os marcadores fiquem por cima
                plotted_labels.add(label)

    ax1.set_title('Error vs Time')
    ax1.set_ylabel('Error')
    ax1.set_ylim(-6, 6) # Set fixed y-axis for error
    ax1.grid(True)
    ax1.legend()

    # Plot 2: PID Component Contributions
    ax2.plot(df['Time'], df['Smoothed_Error'], label='Smoothed_Error', color='green', linewidth=2.5)
    ax2.set_title('Smoothed_Error vs Time')
    ax2.set_ylabel('Smoothed_Error')
    #ax2.set_ylim(-6, 6) # Set fixed y-axis for error
    ax2.grid(True)
    ax2.legend()

    # Plot 3: Total PID Output vs. Time
    ax3.plot(df['Time'], df['PID'], label='Total PID Output', color='red')
    ax3.set_title('PID Output vs Time')
    ax3.set_ylabel('PID Value')
    ax3.grid(True)
    ax3.legend()

    # Plot 4: Motor Velocities vs. Time
    ax4.plot(df['Time'], df['Velocidade Direita'], label='Velocidade Direita', color='purple')
    ax4.plot(df['Time'], df['Velocidade Esquerda'], label='Velocidade Esquerda', color='orange')
    ax4.set_title('Velocidade dos Motores vs. Tempo')
    ax4.set_xlabel('Time (ms)')
    ax4.set_ylabel('Velocidade')
    ax4.grid(True)
    ax4.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
def analyze_performance_and_suggest_pid(df, Kp, Ki, Kd):
    """
    Analisa o desempenho com base nos dados de erro e sugere ajustes de PID
    seguindo a lógica da tabela de referência.
    
    Args:
        df (pd.DataFrame): DataFrame com as colunas 'Time' e 'Error'.
        Kp (float): Ganho Proporcional atual.
        Ki (float): Ganho Integral atual.
        Kd (float): Ganho Derivativo atual.
    """
    print("\n--- Análise de Desempenho & Sugestões de Ajuste (Baseado na Tabela) ---")
    
    error = df['Error'].to_numpy()
    
    # 1. Calcular métricas de desempenho
    mae = np.mean(np.abs(error))  # Erro Médio Absoluto
    max_abs_error = np.max(np.abs(error)) # Erro máximo absoluto (overshoot/undershoot)
    
    # 2. Detectar oscilações usando a busca de picos
    # Encontrar picos (overshoots) e vales (undershoots)
    peaks, _ = find_peaks(error, prominence=0.5)  # A proeminência ajuda a filtrar ruído
    troughs, _ = find_peaks(-error, prominence=0.5)
    num_oscillations = len(peaks) + len(troughs)
    
    print(f"Erro Médio Absoluto (MAE): {mae:.4f}")
    print(f"Máximo Erro Absoluto (Overshoot/Undershoot): {max_abs_error:.4f}")
    print(f"Detectadas {num_oscillations} oscilações significativas (picos/vales).")

    # 3. Fornecer sugestões de ajuste com base nas heurísticas da tabela
    print("\nSugestões:")
    
    suggestion_made = False

    # Comportamento: "Oscila muito em torno da linha (zig-zag)"
    if num_oscillations > 8: # Limiar mais alto para oscilação clara
        print("- Comportamento Detectado: O robô oscila muito em torno da linha (zig-zag).")
        print(f"  - Ação Recomendada: Diminua o valor de Kp (atual: {Kp}). Um Kp muito alto causa reações exageradas.")
        suggestion_made = True

    # Comportamento: "Corrige devagar e não consegue seguir bem a linha"
    if mae > 1.2 and num_oscillations < 5:
        print("- Comportamento Detectado: O robô corrige devagar e não segue a linha com precisão.")
        print(f"  - Ação Recomendada: Aumente o valor de Kp (atual: {Kp}). Um Kp muito baixo gera correções fracas.")
        suggestion_made = True

    # Comportamento: "Erro pequeno persiste por muito tempo"
    if mae > 0.2 and mae < 1.0 and not (num_oscillations > 5):
        print("- Comportamento Detectado: Um erro pequeno parece persistir, não centralizando perfeitamente.")
        print(f"  - Ação Recomendada: Aumente Ki (atual: {Ki}) para corrigir erros acumulados lentamente.")
        suggestion_made = True

    # Comportamento: "Começa a oscilar após um tempo seguindo bem" (Instabilidade do Integral)
    if num_oscillations > 5 and Ki > 0: # Se está oscilando e Ki está ativo
        print("- Causa Possível da Oscilação: O termo integral (Ki) pode estar acumulando erro demais.")
        print(f"  - Ação Recomendada: Diminua Ki (atual: {Ki}) para reduzir a instabilidade causada pelo erro acumulado.")
        suggestion_made = True

    # Comportamento: "Reação muito abrupta a mudanças rápidas na linha"
    if max_abs_error > 2.5: # Overshoot significativo
        print("- Comportamento Detectado: Reação muito abrupta a mudanças, com overshoot/undershoot elevado.")
        print(f"  - Ação Recomendada: Aumente Kd (atual: {Kd}) para suavizar a resposta e 'frear' mudanças bruscas.")
        suggestion_made = True

    # Comportamento: "Resposta lenta a mudanças rápidas"
    if mae > 1.0 and Kd > 0:
        # Esta é uma sugestão de ajuste fino, pode aparecer junto com a de "aumentar Kp".
        print("- Ajuste Fino para Curvas: Se o robô parece 'amortecido' demais e lento para iniciar uma curva.")
        print(f"  - Ação Recomendada: Considere diminuir Kd (atual: {Kd}) para permitir uma reação mais rápida a desvios.")
        suggestion_made = True

    # Mensagem padrão se nenhum comportamento claro for detectado
    if not suggestion_made:
        print("- O desempenho parece razoável. O ajuste fino pode envolver pequenas alterações nos parâmetros atuais.")

    print("\nNota: Estas são sugestões gerais. Os valores ótimos dependem da dinâmica do robô e da pista.")

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

        try:
            # Lê o CSV, assumindo que a primeira linha é o cabeçalho (header=0).
            # Isso resolve o TypeError ao impedir que o cabeçalho seja lido como dados.
            # O pandas irá nomear as colunas como 'Time', 'Error', 'Challenge' a partir do arquivo.
            df = pd.read_csv(selected_file, header=0)

            # Garante que a coluna 'Challenge' exista, mesmo que o arquivo original
            # tivesse apenas 2 colunas (o pandas não a criaria a partir do cabeçalho).
            if 'Challenge' not in df.columns:
                df['Challenge'] = 0

            # Preenche valores ausentes (NaN) na coluna 'Challenge' com 0 e converte para inteiro.
            # Esta é a forma moderna que corrige o FutureWarning e garante o tipo de dado correto.
            df['Challenge'] = df['Challenge'].fillna(0).astype(int)
        except pd.errors.EmptyDataError:
            print(f"Error: O arquivo de log '{basename}' está vazio.")
            input("Pressione Enter para continuar...")
            continue
        except Exception as e:
            print(f"Ocorreu um erro ao ler o arquivo '{basename}': {e}")
            input("Pressione Enter para continuar...")
            continue

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


def run_delete_all_flow(log_dir):
    """Handles the deletion of all log files in the directory."""
    clear_screen()
    log_files = find_log_files(log_dir)
    if not log_files:
        print(f"\nNo .TXT log files found in '{log_dir}' to delete.")
        input("Press Enter to return to the main menu...")
        return

    print(f"Found {len(log_files)} .TXT files in '{log_dir}'.")
    try:
        confirm = input(f"Are you sure you want to permanently delete ALL {len(log_files)} .TXT files in this directory? (y/n): ").lower()
        if confirm == 'y':
            deleted_count = 0
            for file_path in log_files:
                try:
                    os.remove(file_path)
                    deleted_count += 1
                except Exception as e:
                    basename = os.path.basename(file_path)
                    print(f"Could not delete {basename}: {e}")
            print(f"\nSuccessfully deleted {deleted_count} files.")
        else:
            print("\nDeletion cancelled.")
    except Exception as e:
        print(f"An error occurred: {e}")
    
    input("Press Enter to return to the main menu...")

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
    log_dir = "E:\\"

    while True:
        clear_screen()
        print("--- Main Menu ---")
        print(f"Current log directory: {log_dir}")
        print("(1) - Analyze log file")
        print("(2) - Delete a specific log file")
        print("(3) - Change log directory")
        print("(4) - Delete ALL .TXT files in directory")
        print("(0) - Exit")
        
        choice = input("Enter your choice: ")

        if choice == '1':
            run_analysis_flow(log_dir)
        elif choice == '2':
            run_deletion_flow(log_dir)
        elif choice == '3':
            log_dir = change_log_directory(log_dir)
        elif choice == '4':
            run_delete_all_flow(log_dir)
        elif choice == '0':
            print("Exiting program.")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()