"""
Programa de aquisição e análise para o titulador potenciométrico automatizado.

Formato esperado da porta serial, uma linha por ponto:
volume_mL,potencial_mV,temperatura_C,tempo_s

Exemplo:
0.5000,132.450,24.81,38.20

Compatível com registrarPonto() do código do ESP32.
"""

import math
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import pandas as pd
import serial
from serial.tools import list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


BAUD_PADRAO = 115200


def interpretar_linha_serial(linha):
    """Converte 'volume,potencial,temperatura,tempo' em quatro floats."""
    linha = linha.strip()
    if not linha:
        return None

    partes = [p.strip() for p in linha.split(",")]
    if len(partes) != 4:
        return None

    try:
        valores = tuple(map(float, partes))
    except ValueError:
        return None

    if not all(math.isfinite(x) for x in valores):
        return None

    return valores


def calcular_analises(V, E):
    """Calcula 1ª derivada, 2ª derivada e estimativas de Veq."""
    resultado = {
        "V_d1": np.array([], dtype=float),
        "dE_dV": np.array([], dtype=float),
        "V_d2": np.array([], dtype=float),
        "d2E_dV2": np.array([], dtype=float),
        "Veq_d1": None,
        "Veq_d2": None,
    }

    if len(V) < 2:
        return resultado

    V_np = np.asarray(V, dtype=float)
    E_np = np.asarray(E, dtype=float)

    dV = np.diff(V_np)
    dE = np.diff(E_np)
    mascara = dV > 0

    if np.count_nonzero(mascara) < 1:
        return resultado

    indices = np.where(mascara)[0]
    dE_dV = dE[mascara] / dV[mascara]
    V_d1 = (V_np[indices] + V_np[indices + 1]) / 2.0

    resultado["V_d1"] = V_d1
    resultado["dE_dV"] = dE_dV

    if len(dE_dV) >= 1:
        indice = int(np.argmax(np.abs(dE_dV)))
        resultado["Veq_d1"] = float(V_d1[indice])

    if len(dE_dV) < 2:
        return resultado

    dV_d1 = np.diff(V_d1)
    dd1 = np.diff(dE_dV)
    mascara2 = dV_d1 > 0

    if np.count_nonzero(mascara2) < 1:
        return resultado

    indices2 = np.where(mascara2)[0]
    d2 = dd1[mascara2] / dV_d1[mascara2]
    V_d2 = (V_d1[indices2] + V_d1[indices2 + 1]) / 2.0

    resultado["V_d2"] = V_d2
    resultado["d2E_dV2"] = d2

    # Procura cruzamento por zero da segunda derivada e escolhe
    # o mais próximo da estimativa obtida pela primeira derivada.
    if len(d2) >= 2:
        candidatos = []
        for i in range(len(d2) - 1):
            y1, y2 = d2[i], d2[i + 1]
            x1, x2 = V_d2[i], V_d2[i + 1]

            if not (np.isfinite(y1) and np.isfinite(y2)):
                continue

            if y1 == 0:
                candidatos.append(float(x1))
            elif y1 * y2 < 0:
                veq = x1 - y1 * (x2 - x1) / (y2 - y1)
                candidatos.append(float(veq))

        if candidatos:
            referencia = resultado["Veq_d1"]
            if referencia is None:
                resultado["Veq_d2"] = candidatos[0]
            else:
                resultado["Veq_d2"] = min(
                    candidatos, key=lambda x: abs(x - referencia)
                )

    return resultado


class TituladorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Titulador Potenciométrico — Aquisição e Análise")
        self.root.geometry("1180x760")
        self.root.minsize(1000, 680)

        # Mesmas variáveis do Código 1 do TCC.
        self.V = []
        self.E = []
        self.T = []
        self.tempo = []

        self.serial = None
        self.thread_serial = None
        self.parar_thread = threading.Event()
        self.fila = queue.Queue()

        self.status_var = tk.StringVar(value="Desconectado")
        self.porta_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD_PADRAO))

        self.n_pontos_var = tk.StringVar(value="0")
        self.volume_var = tk.StringVar(value="—")
        self.potencial_var = tk.StringVar(value="—")
        self.temperatura_var = tk.StringVar(value="—")
        self.tempo_var = tk.StringVar(value="—")
        self.veq1_var = tk.StringVar(value="—")
        self.veq2_var = tk.StringVar(value="—")

        self._montar_interface()
        self.atualizar_portas()
        self.root.after(100, self.processar_fila)
        self.root.protocol("WM_DELETE_WINDOW", self.fechar)

    def _montar_interface(self):
        topo = ttk.Frame(self.root, padding=10)
        topo.pack(fill="x")

        ttk.Label(topo, text="Porta:").grid(row=0, column=0, padx=(0, 5))
        self.combo_porta = ttk.Combobox(
            topo, textvariable=self.porta_var, width=18, state="readonly"
        )
        self.combo_porta.grid(row=0, column=1, padx=(0, 8))

        ttk.Button(topo, text="Atualizar portas", command=self.atualizar_portas).grid(
            row=0, column=2, padx=(0, 14)
        )

        ttk.Label(topo, text="Baud:").grid(row=0, column=3, padx=(0, 5))
        self.combo_baud = ttk.Combobox(
            topo,
            textvariable=self.baud_var,
            width=10,
            values=("9600", "57600", "115200", "230400"),
        )
        self.combo_baud.grid(row=0, column=4, padx=(0, 10))

        self.btn_conectar = ttk.Button(topo, text="Conectar", command=self.alternar_conexao)
        self.btn_conectar.grid(row=0, column=5, padx=(0, 10))

        ttk.Button(topo, text="Limpar dados", command=self.limpar_dados).grid(
            row=0, column=6, padx=(0, 10)
        )
        ttk.Button(topo, text="Salvar CSV", command=self.salvar_csv).grid(
            row=0, column=7, padx=(0, 10)
        )
        ttk.Label(topo, textvariable=self.status_var).grid(
            row=0, column=8, sticky="w", padx=(10, 0)
        )

        corpo = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        corpo.pack(fill="both", expand=True)

        painel = ttk.LabelFrame(corpo, text="Dados da titulação", padding=12)
        painel.pack(side="left", fill="y", padx=(0, 10))

        self._campo_status(painel, "Pontos:", self.n_pontos_var)
        self._campo_status(painel, "Volume (mL):", self.volume_var)
        self._campo_status(painel, "Potencial (mV):", self.potencial_var)
        self._campo_status(painel, "Temperatura (°C):", self.temperatura_var)
        self._campo_status(painel, "Tempo (s):", self.tempo_var)

        ttk.Separator(painel, orient="horizontal").pack(fill="x", pady=12)
        ttk.Label(painel, text="Volume de equivalência").pack(anchor="w", pady=(0, 8))
        self._campo_status(painel, "1ª derivada (mL):", self.veq1_var)
        self._campo_status(painel, "2ª derivada (mL):", self.veq2_var)

        ttk.Separator(painel, orient="horizontal").pack(fill="x", pady=12)
        ttk.Label(
            painel,
            text=(
                "Formato esperado:\n"
                "volume,potencial,temperatura,tempo\n\n"
                "Exemplo:\n"
                "0.5000,132.450,24.81,38.20"
            ),
            justify="left",
        ).pack(anchor="w")

        self.notebook = ttk.Notebook(corpo)
        self.notebook.pack(side="right", fill="both", expand=True)

        self.fig_curva, self.ax_curva, self.canvas_curva = self._criar_grafico(
            "Curva de titulação", "Volume de titulante (mL)", "Potencial (mV)"
        )
        self.fig_d1, self.ax_d1, self.canvas_d1 = self._criar_grafico(
            "Primeira derivada", "Volume de titulante (mL)", "dE/dV (mV/mL)"
        )
        self.fig_d2, self.ax_d2, self.canvas_d2 = self._criar_grafico(
            "Segunda derivada", "Volume de titulante (mL)", "d²E/dV²"
        )

    def _campo_status(self, pai, rotulo, variavel):
        linha = ttk.Frame(pai)
        linha.pack(fill="x", pady=3)
        ttk.Label(linha, text=rotulo, width=20).pack(side="left")
        ttk.Label(linha, textvariable=variavel).pack(side="left")

    def _criar_grafico(self, titulo, xlabel, ylabel):
        aba = ttk.Frame(self.notebook)
        self.notebook.add(aba, text=titulo)

        fig = Figure(figsize=(7.5, 5.6), dpi=100)
        ax = fig.add_axes([0.12, 0.12, 0.84, 0.80])
        ax.set_title(titulo)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)

        canvas = FigureCanvasTkAgg(fig, master=aba)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        canvas.draw()
        return fig, ax, canvas

    def atualizar_portas(self):
        portas = [p.device for p in list_ports.comports()]
        self.combo_porta["values"] = portas
        if self.porta_var.get() not in portas:
            self.porta_var.set(portas[0] if portas else "")

    def alternar_conexao(self):
        if self.serial is not None and self.serial.is_open:
            self.desconectar()
        else:
            self.conectar()

    def conectar(self):
        porta = self.porta_var.get().strip()
        if not porta:
            messagebox.showwarning("Porta serial", "Nenhuma porta serial foi selecionada.")
            return

        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Baud rate", "Informe um baud rate válido.")
            return

        try:
            self.serial = serial.Serial(porta, baudrate=baud, timeout=0.5)
        except serial.SerialException as exc:
            messagebox.showerror("Erro de conexão", str(exc))
            self.serial = None
            return

        self.parar_thread.clear()
        self.thread_serial = threading.Thread(target=self._loop_serial, daemon=True)
        self.thread_serial.start()
        self.status_var.set(f"Conectado em {porta} @ {baud}")
        self.btn_conectar.config(text="Desconectar")

    def desconectar(self):
        self.parar_thread.set()
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except serial.SerialException:
                pass
        self.serial = None
        self.status_var.set("Desconectado")
        self.btn_conectar.config(text="Conectar")

    def _loop_serial(self):
        while not self.parar_thread.is_set():
            try:
                if self.serial is None or not self.serial.is_open:
                    break

                bruto = self.serial.readline()
                if not bruto:
                    continue

                linha = bruto.decode("utf-8", errors="replace").strip()
                ponto = interpretar_linha_serial(linha)
                if ponto is not None:
                    self.fila.put(("ponto", ponto))

            except serial.SerialException as exc:
                self.fila.put(("erro_serial", str(exc)))
                break
            except Exception as exc:
                self.fila.put(("erro", str(exc)))

    def processar_fila(self):
        recebeu = False

        while True:
            try:
                tipo, dado = self.fila.get_nowait()
            except queue.Empty:
                break

            if tipo == "ponto":
                volume, potencial, temperatura, instante = dado

                if self.V and (
                    volume == self.V[-1]
                    and potencial == self.E[-1]
                    and temperatura == self.T[-1]
                    and instante == self.tempo[-1]
                ):
                    continue

                self.V.append(volume)
                self.E.append(potencial)
                self.T.append(temperatura)
                self.tempo.append(instante)
                recebeu = True

            elif tipo == "erro_serial":
                self.desconectar()
                messagebox.showerror("Erro serial", dado)
            elif tipo == "erro":
                self.status_var.set(f"Aviso: {dado}")

        if recebeu:
            self.atualizar_interface()

        self.root.after(100, self.processar_fila)

    def atualizar_interface(self):
        self.n_pontos_var.set(str(len(self.V)))

        if self.V:
            self.volume_var.set(f"{self.V[-1]:.4f}")
            self.potencial_var.set(f"{self.E[-1]:.3f}")
            self.temperatura_var.set(f"{self.T[-1]:.2f}")
            self.tempo_var.set(f"{self.tempo[-1]:.2f}")
        else:
            self.volume_var.set("—")
            self.potencial_var.set("—")
            self.temperatura_var.set("—")
            self.tempo_var.set("—")

        analise = calcular_analises(self.V, self.E)
        veq1 = analise["Veq_d1"]
        veq2 = analise["Veq_d2"]
        self.veq1_var.set("—" if veq1 is None else f"{veq1:.4f}")
        self.veq2_var.set("—" if veq2 is None else f"{veq2:.4f}")

        self._plot_curva()
        self._plot_d1(analise)
        self._plot_d2(analise)

    def _plot_curva(self):
        self.ax_curva.clear()
        self.ax_curva.set_title("Curva de titulação")
        self.ax_curva.set_xlabel("Volume de titulante (mL)")
        self.ax_curva.set_ylabel("Potencial (mV)")
        self.ax_curva.grid(True, alpha=0.25)
        if self.V:
            self.ax_curva.plot(self.V, self.E, marker="o")
        self.canvas_curva.draw_idle()

    def _plot_d1(self, analise):
        self.ax_d1.clear()
        self.ax_d1.set_title("Primeira derivada")
        self.ax_d1.set_xlabel("Volume de titulante (mL)")
        self.ax_d1.set_ylabel("dE/dV (mV/mL)")
        self.ax_d1.grid(True, alpha=0.25)

        V_d1 = analise["V_d1"]
        d1 = analise["dE_dV"]
        if len(V_d1):
            self.ax_d1.plot(V_d1, d1, marker="o")
            if analise["Veq_d1"] is not None:
                self.ax_d1.axvline(analise["Veq_d1"], linestyle="--")
        self.canvas_d1.draw_idle()

    def _plot_d2(self, analise):
        self.ax_d2.clear()
        self.ax_d2.set_title("Segunda derivada")
        self.ax_d2.set_xlabel("Volume de titulante (mL)")
        self.ax_d2.set_ylabel("d²E/dV²")
        self.ax_d2.grid(True, alpha=0.25)
        self.ax_d2.axhline(0, linewidth=1)

        V_d2 = analise["V_d2"]
        d2 = analise["d2E_dV2"]
        if len(V_d2):
            self.ax_d2.plot(V_d2, d2, marker="o")
            if analise["Veq_d2"] is not None:
                self.ax_d2.axvline(analise["Veq_d2"], linestyle="--")
        self.canvas_d2.draw_idle()

    def limpar_dados(self):
        if self.V:
            if not messagebox.askyesno(
                "Limpar dados",
                "Deseja apagar os pontos atualmente armazenados na interface?",
            ):
                return

        self.V.clear()
        self.E.clear()
        self.T.clear()
        self.tempo.clear()
        self.n_pontos_var.set("0")
        self.veq1_var.set("—")
        self.veq2_var.set("—")
        self.atualizar_interface()

    def salvar_csv(self):
        if not self.V:
            messagebox.showinfo("Salvar CSV", "Ainda não há dados para salvar.")
            return

        caminho = filedialog.asksaveasfilename(
            title="Salvar dados da titulação",
            defaultextension=".csv",
            filetypes=[("Arquivo CSV", "*.csv"), ("Todos os arquivos", "*.*")],
        )
        if not caminho:
            return

        dados = pd.DataFrame(
            {
                "Volume_mL": self.V,
                "Potencial_mV": self.E,
                "Temperatura_C": self.T,
                "Tempo_s": self.tempo,
            }
        )

        try:
            dados.to_csv(caminho, index=False)
        except Exception as exc:
            messagebox.showerror("Erro ao salvar", str(exc))
            return

        messagebox.showinfo("Salvar CSV", "Arquivo salvo com sucesso.")

    def fechar(self):
        self.desconectar()
        self.root.destroy()


def main():
    root = tk.Tk()
    TituladorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
