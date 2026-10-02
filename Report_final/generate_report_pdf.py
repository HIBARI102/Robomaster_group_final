"""
RoboMaster EP Autonomous Maze Exploration & Precision Firing System
PDF Technical Report Generator
Author: นายคุณัชญ์ ทวีรัตน์ 6810110038
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pythainlp

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import cm, mm
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, KeepTogether, PageBreak, HRFlowable
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

# Configure fonts
FONT_REGULAR = "Leelawadee"
FONT_BOLD = "Leelawadee-Bold"

try:
    pdfmetrics.registerFont(TTFont('Leelawadee', 'C:/Windows/Fonts/leelawad.ttf'))
    pdfmetrics.registerFont(TTFont('Leelawadee-Bold', 'C:/Windows/Fonts/leelawdb.ttf'))
except Exception as e:
    print(f"Leelawadee font error: {e}, falling back to Tahoma")
    pdfmetrics.registerFont(TTFont('Leelawadee', 'C:/Windows/Fonts/tahoma.ttf'))
    pdfmetrics.registerFont(TTFont('Leelawadee-Bold', 'C:/Windows/Fonts/tahomabd.ttf'))
    FONT_REGULAR = "Leelawadee"
    FONT_BOLD = "Leelawadee-Bold"

def th(text):
    """Tokenize Thai text and insert zero-width space for perfect ReportLab wrapping"""
    if not text:
        return ""
    words = pythainlp.word_tokenize(text, engine="newmm")
    return "\u200b".join(words)

# Color Palette (Modern Executive Theme)
PRIMARY = colors.HexColor("#0F2744")      # Deep Navy
SECONDARY = colors.HexColor("#1D4ED8")    # Vibrant Blue
ACCENT = colors.HexColor("#0284C7")       # Cyan
DARK_TEXT = colors.HexColor("#1E293B")    # Slate 800
LIGHT_BG = colors.HexColor("#F8FAFC")     # Slate 50
CARD_BG = colors.HexColor("#F1F5F9")      # Slate 100
BORDER_COLOR = colors.HexColor("#CBD5E1") # Slate 300
SUCCESS_BG = colors.HexColor("#ECFDF5")   # Emerald 50
SUCCESS_BORDER = colors.HexColor("#10B981") # Emerald 500
ALERT_BG = colors.HexColor("#FEF2F2")     # Red 50
ALERT_BORDER = colors.HexColor("#F87171") # Red 400

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(OUTPUT_DIR, "report_assets")
os.makedirs(ASSETS_DIR, exist_ok=True)


# -------------------------------------------------------------
# HIGH RESOLUTION DIAGRAM GENERATORS
# -------------------------------------------------------------

def generate_diagram_system_architecture():
    """Generate System Workflow Flowchart"""
    fig, ax = plt.subplots(figsize=(10, 3.8), dpi=300)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')

    fig.patch.set_facecolor('#F8FAFC')
    ax.set_facecolor('#F8FAFC')

    def draw_box(x, y, w, h, title, subtitle, bg_col, border_col):
        rect = patches.FancyBboxPatch(
            (x, y), w, h,
            boxstyle="round,pad=1.2,rounding_size=2.0",
            facecolor=bg_col, edgecolor=border_col, linewidth=1.5
        )
        ax.add_patch(rect)
        ax.text(x + w/2, y + h*0.64, title, ha='center', va='center',
                fontsize=9.2, fontweight='bold', color='#0F2744', fontfamily='sans-serif')
        ax.text(x + w/2, y + h*0.28, subtitle, ha='center', va='center',
                fontsize=7.4, color='#475569', fontfamily='sans-serif')

    def draw_arrow(x1, y1, x2, y2, text=""):
        ax.annotate(
            "", xy=(x2, y2), xytext=(x1, y1),
            arrowprops=dict(arrowstyle="->,head_width=0.35,head_length=0.45",
                            color="#1D4ED8", lw=1.8)
        )
        if text:
            ax.text((x1+x2)/2, (y1+y2)/2 + 2.5, text, ha='center', va='center',
                    fontsize=7, color="#1E40AF", fontweight='bold')

    # Phase 1
    draw_box(2, 54, 28, 38, "Phase 1: Calibration", "Chassis 60cm, ToF & Sharp IR", "#EFF6FF", "#3B82F6")
    draw_box(2, 8, 28, 38, "Vision Tuning", "HSV Dual-Red, Mask & Edge", "#EFF6FF", "#3B82F6")
    draw_arrow(16, 54, 16, 46)

    # Phase 2
    draw_box(36, 50, 30, 42, "Phase 2: Round 1 SLAM", "Grid 60x60cm, 4-Way Wall Sensing\nDFS Search + BFS Backtracking", "#F0FDF4", "#22C55E")
    draw_box(36, 8, 30, 34, "Adaptive Firing Engine", "Closed-Loop PID Servoing\nDynamic ROI Masking", "#FEF3C7", "#F59E0B")
    draw_arrow(30, 73, 36, 71)
    draw_arrow(30, 27, 36, 25)
    draw_arrow(51, 50, 51, 42)

    # Phase 3
    draw_box(72, 50, 26, 42, "Mission Artifacts", "Map_Data_Round1.json\nTelemetry CSV, Target Images", "#F5F3FF", "#8B5CF6")
    draw_box(72, 8, 26, 34, "Phase 3: Round 2", "Global Shortest Path Navigation\nRapid Precision Firing", "#FDF2F8", "#EC4899")
    draw_arrow(66, 71, 72, 71)
    draw_arrow(85, 50, 85, 42)

    plt.tight_layout()
    path = os.path.join(ASSETS_DIR, "system_architecture.png")
    plt.savefig(path, bbox_inches='tight', dpi=300)
    plt.close()
    return path


def generate_diagram_calibration():
    """Generate Sensor Calibration Curves"""
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.2), dpi=300)
    fig.patch.set_facecolor('#F8FAFC')

    for ax in axs:
        ax.set_facecolor('#FFFFFF')
        ax.grid(True, linestyle=':', alpha=0.6, color='#94A3B8')
        for spine in ax.spines.values():
            spine.set_color('#CBD5E1')

    # 1. ToF Sensor Curve
    x_raw = np.linspace(10, 180, 100)
    y_tof = 0.000125 * x_raw**2 + 0.9854 * x_raw - 0.45
    axs[0].plot(x_raw, y_tof, color='#1D4ED8', lw=2, label='Fit: y = 0.000125x² + 0.9854x - 0.45')
    axs[0].plot(x_raw, x_raw, color='#EF4444', linestyle='--', alpha=0.6, label='Ideal: y = x')
    axs[0].set_title("1. Gimbal ToF Calibration\n(R² = 0.9992, Range 10-180 cm)", fontsize=8.5, fontweight='bold', color='#0F2744')
    axs[0].set_xlabel("Raw Sensor Distance (cm)", fontsize=7.5)
    axs[0].set_ylabel("True Calibrated Distance (cm)", fontsize=7.5)
    axs[0].legend(fontsize=6.5, loc='upper left')

    # 2. Sharp IR Sensor Curve
    adc = np.linspace(120, 850, 100)
    dist_sharp = 0.00004512 * adc**2 - 0.065412 * adc + 38.5412
    axs[1].plot(adc, dist_sharp, color='#059669', lw=2, label='Left Sharp: ADC → Distance cm')
    axs[1].axhline(11.0, color='#DC2626', linestyle='--', lw=1.5, label='Guardrail Threshold (11 cm)')
    axs[1].fill_between(adc, 0, 11.0, color='#FEE2E2', alpha=0.4, label='Emergency Strafe Zone')
    axs[1].set_title("2. Sharp IR Sensor Conversion\n(Analog ADC to Distance cm)", fontsize=8.5, fontweight='bold', color='#0F2744')
    axs[1].set_xlabel("Raw ADC Value (10-bit: 0-1023)", fontsize=7.5)
    axs[1].set_ylabel("Measured Distance (cm)", fontsize=7.5)
    axs[1].set_ylim(0, 35)
    axs[1].legend(fontsize=6.5, loc='upper right')

    # 3. Chassis Ramp-Down Deceleration Profile
    dist = np.linspace(0, 60, 100)
    speed = np.where(dist < 42, 0.35, 0.35 - (dist - 42) / 18 * (0.35 - 0.08))
    axs[2].plot(dist, speed, color='#7C3AED', lw=2, label='Velocity Profile (m/s)')
    axs[2].axvline(42, color='#D97706', linestyle=':', lw=1.5, label='Decel Point (18 cm to goal)')
    axs[2].scatter([60], [0.0], color='#DC2626', s=40, zorder=5, label='Active Brake @ 60cm')
    axs[2].set_title("3. Chassis Ramp-down Motion\n(Smooth Decel & Yaw Holding)", fontsize=8.5, fontweight='bold', color='#0F2744')
    axs[2].set_xlabel("Travel Distance in Cell (cm)", fontsize=7.5)
    axs[2].set_ylabel("Linear Velocity (m/s)", fontsize=7.5)
    axs[2].legend(fontsize=6.5, loc='upper right')

    plt.tight_layout()
    path = os.path.join(ASSETS_DIR, "calibration_curves.png")
    plt.savefig(path, bbox_inches='tight', dpi=300)
    plt.close()
    return path


def generate_diagram_vision_shapes():
    """Generate Target Shapes and Geometric Decision Diagram"""
    fig, axs = plt.subplots(1, 4, figsize=(10, 2.3), dpi=300)
    fig.patch.set_facecolor('#F8FAFC')

    shapes = [
        ("CIRCLE", "red", "Ø 7 cm", "Vertices >= 5\nExtent < 0.83 (Theory: 0.785)"),
        ("SQUARE", "green", "7 x 7 cm", "Vertices = 4\nAspect Ratio: 0.85 - 1.18"),
        ("RECT_H", "blue", "9 x 6 cm", "Vertices = 4\nAspect Ratio > 1.18"),
        ("RECT_V", "gold", "6 x 9 cm", "Vertices = 4\nAspect Ratio < 0.85")
    ]

    for ax, (name, col, dim, logic) in zip(axs, shapes):
        ax.set_facecolor('#FFFFFF')
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.axis('off')
        
        card = patches.FancyBboxPatch((-1.4, -1.4), 2.8, 2.8, boxstyle="round,pad=0.08",
                                      facecolor='#F1F5F9', edgecolor='#CBD5E1', lw=1)
        ax.add_patch(card)

        if name == "CIRCLE":
            patch = patches.Circle((0, 0.35), 0.52, facecolor='#DC2626', edgecolor='#991B1B', lw=1.5)
        elif name == "SQUARE":
            patch = patches.Rectangle((-0.48, -0.13), 0.96, 0.96, facecolor='#16A34A', edgecolor='#15803D', lw=1.5)
        elif name == "RECT_H":
            patch = patches.Rectangle((-0.72, 0.05), 1.44, 0.6, facecolor='#2563EB', edgecolor='#1E40AF', lw=1.5)
        elif name == "RECT_V":
            patch = patches.Rectangle((-0.32, -0.28), 0.64, 1.25, facecolor='#EAB308', edgecolor='#CA8A04', lw=1.5)
        ax.add_patch(patch)

        ax.text(0, -0.62, f"{name} ({dim})", ha='center', va='center', fontsize=8, fontweight='bold', color='#0F2744')
        ax.text(0, -1.02, logic, ha='center', va='center', fontsize=6.8, color='#475569')

    plt.tight_layout()
    path = os.path.join(ASSETS_DIR, "vision_shapes.png")
    plt.savefig(path, bbox_inches='tight', dpi=300)
    plt.close()
    return path


def generate_diagram_slam_grid():
    """Generate SLAM Grid Map & Navigation Illustration"""
    fig, ax = plt.subplots(figsize=(6, 5), dpi=300)
    fig.patch.set_facecolor('#F8FAFC')
    ax.set_facecolor('#0F172A')

    grid_size = 4
    for i in range(grid_size + 1):
        ax.axhline(i, color='#334155', lw=1, linestyle='--')
        ax.axvline(i, color='#334155', lw=1, linestyle='--')

    explored = [(0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (2, 3), (3, 3)]
    for x, y in explored:
        rect = patches.Rectangle((x+0.05, y+0.05), 0.9, 0.9, facecolor='#065F46', alpha=0.55)
        ax.add_patch(rect)
        ax.text(x+0.5, y+0.5, f"({x},{y})", ha='center', va='center', color='#6EE7B7', fontsize=7.2, fontweight='bold')

    walls = [
        ((0, 1), 0), ((1, 2), 0), ((2, 3), 0), ((3, 4), 0),
        ((0, 1), 2), ((2, 3), 2),
        ((1, 2), 3), ((3, 4), 3),
        ((0, 4), 4),
        (0, (0, 4)), (4, (0, 4)),
        (1, (0, 1)), (2, (1, 2)), (3, (2, 3))
    ]
    for w in walls:
        if isinstance(w[0], tuple):
            ax.plot([w[0][0], w[0][1]], [w[1], w[1]], color='#F8FAFC', lw=3.5, solid_capstyle='round')
        else:
            ax.plot([w[0], w[0]], [w[1][0], w[1][1]], color='#F8FAFC', lw=3.5, solid_capstyle='round')

    path_x = [0.5, 0.5, 1.5, 1.5, 2.5, 2.5, 3.5]
    path_y = [0.5, 1.5, 1.5, 2.5, 2.5, 3.5, 3.5]
    ax.plot(path_x, path_y, color='#38BDF8', lw=2, linestyle='-', marker='o', markersize=4, label='Exploration Trail (DFS)')
    ax.plot(3.5, 3.5, marker='^', markersize=10, color='#F59E0B', label='Robot Pose (Heading 0°)')
    ax.scatter([0.5, 2.5], [2.0, 2.0], color='#EF4444', s=60, marker='s', zorder=6, label='Detected Target')

    ax.set_xlim(-0.2, 4.2)
    ax.set_ylim(-0.2, 4.2)
    ax.set_aspect('equal')
    ax.set_xticks(range(5))
    ax.set_yticks(range(5))
    ax.tick_params(colors='#94A3B8', labelsize=7)
    ax.set_title("Grid SLAM 2D Representation (60x60 cm per Cell)", color='#F8FAFC', fontsize=9, fontweight='bold', pad=8)
    ax.legend(loc='lower right', fontsize=6.5, facecolor='#1E293B', edgecolor='#475569', labelcolor='#F8FAFC')

    plt.tight_layout()
    path = os.path.join(ASSETS_DIR, "slam_grid_demo.png")
    plt.savefig(path, bbox_inches='tight', dpi=300)
    plt.close()
    return path


# -------------------------------------------------------------
# NUMBERED CANVAS
# -------------------------------------------------------------

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        self.saveState()
        page_w, page_h = A4
        
        # Cover page (page 1) special styling
        if self._pageNumber == 1:
            self.setFillColor(PRIMARY)
            self.rect(0, page_h - 16*mm, page_w, 16*mm, stroke=0, fill=1)
            self.setFillColor(SECONDARY)
            self.rect(0, page_h - 18*mm, page_w, 2*mm, stroke=0, fill=1)
            
            self.setFillColor(PRIMARY)
            self.rect(0, 0, page_w, 13*mm, stroke=0, fill=1)
            self.setFillColor(colors.white)
            self.setFont(FONT_REGULAR, 8)
            self.drawCentredString(page_w / 2.0, 5*mm, "RoboMaster EP Engineering Technical Documentation | Automated Maze SLAM & Precision Firing")
            self.restoreState()
            return

        # Running Header (Page 2+)
        self.setFont(FONT_REGULAR, 8)
        self.setFillColor(colors.HexColor("#64748B"))
        self.drawString(18*mm, page_h - 12*mm, "รายงานคำอธิบายหลักการทำงานของชุดโค้ดโปรแกรมหุ่นยนต์ RoboMaster EP")
        
        self.setStrokeColor(BORDER_COLOR)
        self.setLineWidth(0.6)
        self.line(18*mm, page_h - 14*mm, page_w - 18*mm, page_h - 14*mm)

        # Running Footer (Page 2+)
        self.line(18*mm, 15*mm, page_w - 18*mm, 15*mm)
        self.setFont(FONT_REGULAR, 8)
        self.setFillColor(colors.HexColor("#64748B"))
        self.drawString(18*mm, 10*mm, "ผู้จัดทำ: นายคุณัชญ์ ทวีรัตน์ (รหัสนักศึกษา 6810110038)")
        
        page_str = f"หน้า {self._pageNumber} จาก {page_count}"
        self.drawRightString(page_w - 18*mm, 10*mm, page_str)
        self.restoreState()


# -------------------------------------------------------------
# MAIN REPORT BUILDER FUNCTION
# -------------------------------------------------------------

def build_pdf():
    pdf_path = os.path.join(OUTPUT_DIR, "RoboMaster_EP_Final_Report.pdf")
    print(f"Generating Technical PDF Report: {pdf_path}")

    # Generate assets
    arch_img = generate_diagram_system_architecture()
    calib_img = generate_diagram_calibration()
    vision_img = generate_diagram_vision_shapes()
    slam_img = generate_diagram_slam_grid()

    # Document Template
    doc = SimpleDocTemplate(
        pdf_path,
        pagesize=A4,
        leftMargin=18*mm,
        rightMargin=18*mm,
        topMargin=18*mm,
        bottomMargin=18*mm
    )

    styles = getSampleStyleSheet()
    
    body_style = ParagraphStyle(
        'ThaiBody',
        fontName=FONT_REGULAR,
        fontSize=9.0,
        leading=13.5,
        textColor=DARK_TEXT,
        spaceAfter=4
    )
    
    body_bold = ParagraphStyle(
        'ThaiBodyBold',
        fontName=FONT_BOLD,
        fontSize=9.0,
        leading=13.5,
        textColor=DARK_TEXT,
        spaceAfter=4
    )

    title_main = ParagraphStyle(
        'ThaiTitleMain',
        fontName=FONT_BOLD,
        fontSize=16.0,
        leading=21,
        textColor=PRIMARY,
        alignment=1,
        spaceAfter=4
    )

    title_sub = ParagraphStyle(
        'ThaiTitleSub',
        fontName=FONT_REGULAR,
        fontSize=9.8,
        leading=14,
        textColor=SECONDARY,
        alignment=1,
        spaceAfter=7
    )

    h1_style = ParagraphStyle(
        'ThaiH1',
        fontName=FONT_BOLD,
        fontSize=12.0,
        leading=16,
        textColor=PRIMARY,
        spaceBefore=7,
        spaceAfter=4,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'ThaiH2',
        fontName=FONT_BOLD,
        fontSize=9.8,
        leading=14,
        textColor=SECONDARY,
        spaceBefore=6,
        spaceAfter=3,
        keepWithNext=True
    )

    callout_text = ParagraphStyle(
        'ThaiCallout',
        fontName=FONT_REGULAR,
        fontSize=8.5,
        leading=12.5,
        textColor=DARK_TEXT
    )

    table_header_style = ParagraphStyle(
        'TableHeader',
        fontName=FONT_BOLD,
        fontSize=8.2,
        leading=11,
        textColor=colors.white,
        alignment=1
    )

    table_cell_style = ParagraphStyle(
        'TableCell',
        fontName=FONT_REGULAR,
        fontSize=7.8,
        leading=11,
        textColor=DARK_TEXT
    )

    table_cell_bold = ParagraphStyle(
        'TableCellBold',
        fontName=FONT_BOLD,
        fontSize=7.8,
        leading=11,
        textColor=DARK_TEXT
    )

    def make_callout(text, bg_color=CARD_BG, border_color=ACCENT):
        p = Paragraph(th(text), callout_text)
        t = Table([[p]], colWidths=[174*mm])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, -1), bg_color),
            ('BOX', (0, 0), (-1, -1), 1, border_color),
            ('LEFTPADDING', (0, 0), (-1, -1), 8),
            ('RIGHTPADDING', (0, 0), (-1, -1), 8),
            ('TOPPADDING', (0, 0), (-1, -1), 5),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 5),
        ]))
        return t

    story = []

    # =========================================================================
    # PAGE 1: TITLE, AUTHOR, CODE OVERVIEW & ARCHITECTURE
    # =========================================================================
    story.append(Spacer(1, 2*mm))
    story.append(Paragraph(th("รายงานสรุปเชิงเทคนิค (Technical Code Architecture Report)"), title_sub))
    story.append(Paragraph(th("คำอธิบายหลักการทำงานของชุดโค้ดโปรแกรมหุ่นยนต์อัตโนมัติ RoboMaster EP"), title_main))
    story.append(Paragraph(th("Autonomous Maze SLAM Navigation, Computer Vision & Precision Firing System"), title_sub))
    story.append(Spacer(1, 1.5*mm))

    # Author Box Table
    author_info = [
        [
            Paragraph(th("<b>ผู้จัดทำรายงาน:</b> นายคุณัชญ์ ทวีรัตน์"), body_style),
            Paragraph(th("<b>รหัสนักศึกษา:</b> 6810110038"), body_style)
        ],
        [
            Paragraph(th("<b>แพลตฟอร์ม:</b> DJI RoboMaster EP + Python SDK"), body_style),
            Paragraph(th("<b>หัวข้อรายงาน:</b> หลักการทำงานของโค้ดระบบนำทาง SLAM และคอมพิวเตอร์วิทัศน์"), body_style)
        ]
    ]
    author_table = Table(author_info, colWidths=[87*mm, 87*mm])
    author_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor("#EFF6FF")),
        ('BOX', (0, 0), (-1, -1), 1, colors.HexColor("#93C5FD")),
        ('INNERGRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#DBEAFE")),
        ('PADDING', (0, 0), (-1, -1), 4.5),
    ]))
    story.append(author_table)
    story.append(Spacer(1, 3*mm))

    story.append(Paragraph(th("1. ภาพรวมและหลักการทำงานของระบบโค้ด (System & Code Overview)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1.2, color=PRIMARY, spaceAfter=5))
    
    exec_summary_text = (
        "รายงานฉบับนี้จัดทำขึ้นเพื่ออธิบายหลักการทำงานเชิงลึกของชุดโปรแกรมภาษา Python สำหรับควบคุมหุ่นยนต์ <b>DJI RoboMaster EP</b> "
        "ในการปฏิบัติภารกิจเขาวงกตขนาดกริดมาตรฐาน 60 x 60 cm โดยโครงสร้างโค้ดถูกแบ่งออกเป็น 3 ส่วนหลักอย่างเป็นระบบ:\n"
        "• <b>การปรับเทียบเซนเซอร์ (Calibration Module)</b>: โค้ดสำหรับคำนวณ Fitting สมการถดถอยพหุนามแปลงค่าดิบของเซนเซอร์ ToF, Sharp IR และระยะก้าวเดินของแชสซี\n"
        "• <b>การตรวจจับภาพและการเล็งยิง (Vision & Firing Engine)</b>: โค้ดประมวลผล OpenCV สกัดสี HSV แบบ Dual-band, กรองสัญญาณรบกวน, จำแนกรูปทรงเรขาคณิต และระบบ Closed-Loop Visual Servoing ควบคุม Gimbal\n"
        "• <b>ระบบนำทางอัตโนมัติ SLAM (Round 1 Autonomous Run)</b>: โค้ดหลัก <code>run_round1.py</code> ผสานการสแกนกำแพง 4 ทิศทาง, อัลกอริทึม DFS Frontier, BFS Dead-End Backtracking และ Sharp IR Safety Guardrail"
    )
    story.append(Paragraph(th(exec_summary_text), body_style))
    story.append(Spacer(1, 2*mm))

    story.append(Image(arch_img, width=174*mm, height=66*mm))
    story.append(Spacer(1, 2*mm))

    # =========================================================================
    # PAGE 2: PHYSICAL SPECS & HARDWARE CALIBRATION
    # =========================================================================
    story.append(PageBreak())
    story.append(Paragraph(th("2. ข้อมูลจำเพาะทางกายภาพและสนาม (Physical & Arena Specifications)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    specs_data = [
        [Paragraph(th("<b>ส่วนประกอบ / โมดูล</b>"), table_header_style),
         Paragraph(th("<b>มิติและค่าพารามิเตอร์ทางกายภาพ</b>"), table_header_style),
         Paragraph(th("<b>หน้าที่และบทบาทในการทำงานของโค้ด</b>"), table_header_style)],
        
        [Paragraph(th("<b>แชสซีและล้อ (Chassis)</b>"), table_cell_bold),
         Paragraph(th("ยาว 33 cm, กว้าง 25 cm, 4 Mecanum Wheels"), table_cell_style),
         Paragraph(th("ขับเคลื่อนแบบ Omnidirectional และรักษาแนวระนาบด้วย Gyro IMU"), table_cell_style)],
        
        [Paragraph(th("<b>Gimbal & Center</b>"), table_cell_bold),
         Paragraph(th("สูงจากพื้น 22 cm, หมุน Yaw & Pitch อิสระ"), table_cell_style),
         Paragraph(th("แพลตฟอร์มหมุนสำหรับเซนเซอร์ ToF, กล้อง และปืนยิง Blaster"), table_cell_style)],

        [Paragraph(th("<b>เซนเซอร์ ToF (Time-of-Flight)</b>"), table_cell_bold),
         Paragraph(th("ยื่นห่างแกนหมุน 7.5 cm, สูงจากแกน 6.5 cm"), table_cell_style),
         Paragraph(th("สแกนกำแพง 4 ทิศทาง (0°, 90°, 180°, -90°) ระยะ Threshold 52 cm"), table_cell_style)],

        [Paragraph(th("<b>กล้องหลัก (Main Camera)</b>"), table_cell_bold),
         Paragraph(th("ยื่นห่างแกนหมุน 7.5 cm, สูงจากแกน 3.5 cm"), table_cell_style),
         Paragraph(th("ตรวจจับสี HSV, รูปทรงเรขาคณิต และ Visual Servoing เล็งยิง"), table_cell_style)],

        [Paragraph(th("<b>Sharp IR Sensors (L/R)</b>"), table_cell_bold),
         Paragraph(th("ขวา: ID 1 Port 1, ซ้าย: ID 2 Port 1"), table_cell_style),
         Paragraph(th("ระบบ Safety Guardrail ป้องกันการครูดชนกำแพงด้านข้าง (< 11 cm)"), table_cell_style)],

        [Paragraph(th("<b>สนามเขาวงกต (Arena Grid)</b>"), table_cell_bold),
         Paragraph(th("ช่องกริด 60x60 cm, กำแพงหนา 7.5 cm, สูง 30 cm"), table_cell_style),
         Paragraph(th("โครงสร้างตารางพิกัด 2 มิติ (gx, gy) สำหรับระบบ SLAM"), table_cell_style)],

        [Paragraph(th("<b>เป้าหมาย (Targets)</b>"), table_cell_bold),
         Paragraph(th("4 รูปทรง (Square, Rect_H, Rect_V, Circle) x 4 สี (R, Y, G, B)"), table_cell_style),
         Paragraph(th("ติดบนกำแพง สูงตามเกณฑ์ ยิงด้วยกระสุนเจล 2 นัดต่อเป้า"), table_cell_style)]
    ]
    specs_table = Table(specs_data, colWidths=[42*mm, 58*mm, 74*mm])
    specs_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), PRIMARY),
        ('GRID', (0, 0), (-1, -1), 0.5, BORDER_COLOR),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, CARD_BG]),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('PADDING', (0, 0), (-1, -1), 3.5),
    ]))
    story.append(specs_table)
    story.append(Spacer(1, 4*mm))

    # SECTION 3: CALIBRATION
    story.append(Paragraph(th("3. หลักการทำงานของโค้ดปรับเทียบเซนเซอร์ (Calibration Module)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    calib_desc = (
        "ความแม่นยำของระบบอัตโนมัติขึ้นอยู่กับความถูกต้องของข้อมูลจากเซนเซอร์ โค้ดในโฟลเดอร์ <code>calibration/</code> "
        "จึงทำการเก็บข้อมูลดิบและคำนวณการถดถอยพหุนาม (Polynomial Regression) เพื่อแปลงสัญญาณดิบเป็นระยะทางจริง:"
    )
    story.append(Paragraph(th(calib_desc), body_style))
    story.append(Spacer(1, 1*mm))

    story.append(Image(calib_img, width=174*mm, height=48*mm))
    story.append(Spacer(1, 2*mm))

    calib_detail_1 = (
        "• <b>3.1 การปรับเทียบระยะการเดินของแชสซี (<code>calibrate_distance.py</code>)</b>: "
        "โค้ดผสานสัญญาณระหว่าง Wheel Encoders และ IMU Attitude ไจโรสโคป ด้วยการควบคุม <b>PID Yaw Holding</b> "
        "(Kp = 1.2, Ki = 0.05, Kd = 0.1) เพื่อล็อคทิศทางมุม 0° ป้องกันการดริฟท์ของล้อ Mecanum "
        "ร่วมกับโปรไฟล์ความเร็วแบบ <b>Ramp-Down Deceleration</b> โดยเริ่มชะลอความเร็วลงเหลือ 0.10 m/s "
        "ที่ระยะ 85% ของช่อง (18 cm สุดท้าย) และสั่ง Active Brake เมื่อถึง 60 cm เพื่อคำนวณ Distance Factor บันทึกลง <code>distance_calib_params.json</code>"
    )
    story.append(Paragraph(th(calib_detail_1), body_style))

    calib_detail_2 = (
        "• <b>3.2 การปรับเทียบเซนเซอร์วัดระยะ ToF บน Gimbal (<code>calibration_ToF.py</code>)</b>: "
        "เซนเซอร์ Time-of-Flight มีความคลาดเคลื่อนแบบไม่เชิงเส้นในระยะ 10 - 180 cm "
        "โค้ดทำการเก็บตัวอย่างละ 100 ค่า นำค่ามัธยฐาน (Median) มาทำ Quadratic Curve Fitting ได้สมการ: "
        "<b>y = 0.000125 x² + 0.9854 x - 0.4500 (R² = 0.9992)</b> บันทึกพารามิเตอร์ลง <code>tof_calib_params.json</code>"
    )
    story.append(Paragraph(th(calib_detail_2), body_style))

    calib_detail_3 = (
        "• <b>3.3 การปรับเทียบเซนเซอร์ Sharp Infrared สำหรับระบบ Safety Guardrail (<code>sharp_calibration.py</code>)</b>: "
        "เซนเซอร์ Sharp IR ด้านข้างซ้ายและขวาอ่านค่าแรงดันไฟฟ้าสัญญาณอนาล็อก (ADC: 0–1023) "
        "แปลงเป็นระยะทางจริง (5 - 30 cm) ผ่านสมการกำลังสองใน <code>calibration_sharp_poly.json</code> "
        "พร้อมทั้งกำหนด Noise Baseline ในที่โล่งเพื่อป้องกันการสั่งงานผิดพลาด โดยหากระยะด้านข้างน้อยกว่า 11 cm "
        "ระบบจะสั่ง Strafe ขับผลักหุ่นยนต์ออกจากกำแพงทันที (0.10 m/s)"
    )
    story.append(Paragraph(th(calib_detail_3), body_style))

    # =========================================================================
    # PAGE 3: COMPUTER VISION & SHAPE CLASSIFICATION
    # =========================================================================
    story.append(PageBreak())
    story.append(Paragraph(th("4. หลักการทำงานของโค้ดตรวจจับภาพและการจำแนกรูปทรง (Computer Vision Module)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    vision_desc = (
        "โค้ดประมวลผลภาพในโฟลเดอร์ <code>Camera_Detection/</code> ทำงานแบบ Real-time บนภาพ BGR จากกล้องหลักผ่านกระบวนการ Pipeline ดังนี้:\n"
        "1. <b>ROI Masking</b>: บังคับใช้หน้ากากไบนารี <code>roi_mask.png</code> เพื่อตัดส่วนรบกวนของแชสซีและขอบล้อออก\n"
        "2. <b>Dual-Band Red HSV Filtering</b>: เนื่องจากสีแดงมีค่า Hue คร่อมรอยต่อ 0°-12° และ 165°-180° "
        "โค้ดจึงสกัดสีแดง 2 ช่วงแล้วรวมด้วย Bitwise-OR: <code>Mask_Red = inRange(HSV, Low1, Up1) OR inRange(HSV, Low2, Up2)</code> "
        "ส่วนสี Yellow, Green, Blue ใช้ Single-band Filtering ปกติ\n"
        "3. <b>Morphological Noise Removal</b>: ผ่านฟังก์ชัน Open และ Close ขนาด Kernel 5 x 5 เพื่อกำจัดจุดรบกวนและเชื่อมเม็ดสีให้ทึบสมบูรณ์\n"
        "4. <b>Hybrid Edge Detection</b>: ผสานขอบจาก Sobel Gradient (G = √(Gx² + Gy²)) ร่วมกับ Canny Edge Detector (50, 150) ด้วย Bitwise-OR\n"
        "5. <b>Geometric Shape Decision Tree</b>: จำแนก 4 รูปทรงด้วยคุณสมบัติทางเรขาคณิต"
    )
    story.append(Paragraph(th(vision_desc), body_style))
    story.append(Spacer(1, 2*mm))

    story.append(Image(vision_img, width=174*mm, height=38*mm))
    story.append(Spacer(1, 2.5*mm))

    shape_logic_box = (
        "<b>เกณฑ์และสมการตัดสินรูปทรงเรขาคณิต (Geometric Classification Criteria):</b><br/>"
        "• <b>Fill Ratio (ความหนาแน่นของเม็ดสี)</b>: NonZero Pixels / (Width x Height) ≥ 40% และ Contour Area ≥ 150 px<br/>"
        "• <b>CIRCLE (วงกลม)</b>: จำนวนจุดยอดโพลีกอน Vertices ≥ 5 และ Extent = Contour Area / (W x H) < 0.83 (ตามทฤษฎีวงกลม π/4 ≈ 0.785)<br/>"
        "• <b>SQUARE (สี่เหลี่ยมจัตุรัส)</b>: Vertices = 4 และ Aspect Ratio (W/H) อยู่ในช่วง [0.85, 1.18]<br/>"
        "• <b>RECT_H (สี่เหลี่ยมผืนผ้าแนวนอน)</b>: Vertices = 4 และ Aspect Ratio (W/H) > 1.18<br/>"
        "• <b>RECT_V (สี่เหลี่ยมผืนผ้าแนวตั้ง)</b>: Vertices = 4 และ Aspect Ratio (W/H) < 0.85"
    )
    story.append(make_callout(shape_logic_box, bg_color=ALERT_BG, border_color=ALERT_BORDER))
    story.append(Spacer(1, 3.5*mm))

    # SECTION 5: TARGET MOCK & ADAPTIVE FIRING
    story.append(Paragraph(th("5. โค้ดจำลองระบบเล็งยิงเป้าหมาย (Target Mock & Adaptive Firing Module)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    mock_desc = (
        "โค้ด <code>test_firing_mock.py</code> ในโฟลเดอร์ <code>Target_mock/</code> พัฒนาขึ้นเพื่อทดสอบวงจรการค้นหา ยืนยัน เล็ง และยิงเป้าหมาย "
        "โดยมีหลักการและฟังก์ชันสำคัญที่นำไปใช้งานในโค้ดหลัก ดังนี้:\n"
        "• <b>Pygame Interactive Rule Configurator</b>: เมทริกซ์ 4 x 4 ให้ผู้ใช้เลือกคู่สีและรูปทรงที่ต้องการยิงแบบ <i>Strict Matching Principle</i> "
        "โค้ดจะไม่ยิงเป้าหมายที่สีตรงแต่รูปทรงไม่ตรง หรือรูปทรงตรงแต่สีไม่ตรงเด็ดขาด\n"
        "• <b>Adaptive Speed Closed-Loop Aiming</b>: ควบคุม Gimbal ด้วย Visual Servoing แปรผันความเร็วตามระยะคลาดเคลื่อนพิกเซล:\n"
        "  - Error รวม E > 120 px -> Gimbal Speed = 540°/s (เคลื่อนที่ความเร็วสูงเข้าหาเป้า)\n"
        "  - 50 < E ≤ 120 px -> Gimbal Speed = 320°/s (เคลื่อนที่ความเร็วปานกลาง)\n"
        "  - E ≤ 50 px -> Gimbal Speed = 140°/s (ความเร็วละเอียด นุ่มนวล ป้องกัน Overshooting)\n"
        "• <b>Gravity Drop Offset</b>: ชดเชยแนวดิ่ง ey = y_target - (y_center - 32 px) เพื่อให้วิถีกระสุนเจลเข้ากลางเป้าหมายอย่างแม่นยำ\n"
        "• <b>Dynamic ROI Masking</b>: เมื่อยิงเป้าหมายเสร็จสิ้น โค้ดจะถมพิกเซลสีดำทับ Bounding Box ของเป้าหมายนั้นขยาย +15 px ทันที ป้องกันการเล็งซ้ำ\n"
        "• <b>Chassis Retreat & Return Maneuver</b>: หากอยู่ชิดกำแพงจนกล้องก้มมองไม่เห็นเป้าหมาย แชสซีจะถอยหลังชั่วคราวเพื่อยิง แล้วขับเคลื่อนกลับพิกัดเดิม"
    )
    story.append(Paragraph(th(mock_desc), body_style))

    # =========================================================================
    # PAGE 4: SLAM NAVIGATION & MESH ARCHITECTURE
    # =========================================================================
    story.append(PageBreak())
    story.append(Paragraph(th("6. โค้ดระบบนำทาง SLAM และการควบคุมการเคลื่อนที่ (Round 1 Main Code)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    slam_intro = (
        "ในสคริปต์หลัก <code>run_round1.py</code> โค้ดควบคุมให้หุ่นยนต์เริ่มต้นจากช่อง (0, 0) และดำเนินการสำรวจแบบอัตโนมัติตลอดเวลาจำกัด 9.5 นาที "
        "โดยผสานระบบ SLAM, การตรวจจับกำแพง 4 ทิศทาง, การยิงเป้าหมาย และการหลบหลีกสิ่งกีดขวาง:"
    )
    story.append(Paragraph(th(slam_intro), body_style))
    story.append(Spacer(1, 2*mm))

    slam_col_text = (
        "<b>6.1 การตรวจจับกำแพง 4 ทิศทาง (4-Way Wall Sensing)</b><br/>"
        "เมื่อเข้าสู่กึ่งกลางช่อง Gimbal จะหมุนสแกน 4 ทิศ (Ahead 0°, Right 90°, Back 180°, Left -90°) "
        "แปลงเป็นทิศสัมบูรณ์ (Heading + Yaw) mod 360 หากระยะ ToF ≤ 52.0 cm "
        "ถือว่ามีกำแพง และซิงค์แผนที่ 2 ทิศทาง (Mutual Sync) ไปยังช่องข้างเคียงทันที<br/><br/>"
        "<b>6.2 อัลกอริทึม SLAM & Path Planning</b><br/>"
        "• <b>DFS Exploration</b>: ให้ความสำคัญกับทิศทางตรงไปข้างหน้าก่อนเพื่อลดการหมุนตัวของแชสซี<br/>"
        "• <b>BFS Dead-End Backtracking</b>: เมื่อพบทางตัน โค้ดใช้ Breadth-First Search คำนวณเส้นทางที่สั้นที่สุดถอยกลับไปยัง Frontier Cell ที่ยังเปิดอยู่<br/>"
        "• <b>Shortest Path Return to Base</b>: เมื่อสำรวจครบทุกช่อง โค้ดใช้ BFS วิ่งกลับ (0, 0) โดยอัตโนมัติ<br/><br/>"
        "<b>6.3 การควบคุมและ Pygame Dashboard</b><br/>"
        "โค้ดแสดงหน้าต่างแสดงผลแผนที่สด 60x60 cm, พิกัดหุ่นยนต์, สถานะกำแพง, เป้าหมายที่ยิงแล้ว, ค่าเซนเซอร์ ToF 4 ทิศ และ Sharp IR แบบ Real-time"
    )
    
    slam_table_layout = Table([
        [Image(slam_img, width=76*mm, height=63*mm),
         Paragraph(th(slam_col_text), body_style)]
    ], colWidths=[80*mm, 94*mm])
    slam_table_layout.setStyle(TableStyle([
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('PADDING', (0, 0), (-1, -1), 2),
    ]))
    story.append(slam_table_layout)
    story.append(Spacer(1, 4*mm))

    # SECTION 7: KEY TECHNOLOGY MATRIX & CONFIG CONSTANTS
    story.append(Paragraph(th("7. ตารางสรุปอัลกอริทึมและเทคนิคที่ใช้ในโค้ด (Algorithm & Method Matrix)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    tech_data = [
        [Paragraph(th("<b>โมดูล / ระบบย่อย</b>"), table_header_style),
         Paragraph(th("<b>อัลกอริทึม / ฟังก์ชันในโค้ด</b>"), table_header_style),
         Paragraph(th("<b>วัตถุประสงค์และประโยชน์เชิงวิศวกรรม</b>"), table_header_style)],

        [Paragraph(th("<b>Chassis Motion Control</b>"), table_cell_bold),
         Paragraph(th("Proportional Yaw Holding + Decel Ramp-down"), table_cell_style),
         Paragraph(th("คุมหุ่นยนต์วิ่งตรงแนว 0° ไม่ส่ายเอียง และหยุดตรงกึ่งกลางช่องกริดพอดี"), table_cell_style)],

        [Paragraph(th("<b>Safety Guardrail</b>"), table_cell_bold),
         Paragraph(th("Sharp IR Quadratic Conversion + Strafe Push"), table_cell_style),
         Paragraph(th("ป้องกันหุ่นยนต์ครูดกำแพงด้านข้างเมื่อระยะ < 11 cm ด้วยแรงผลัก 0.10 m/s"), table_cell_style)],

        [Paragraph(th("<b>Wall Detection</b>"), table_cell_bold),
         Paragraph(th("Gimbal 4-Way Scan + ToF Quadratic Compensation"), table_cell_style),
         Paragraph(th("ตรวจสอบกำแพง 4 ทิศทางด้วยระยะ Threshold 52 cm แม่นยำ R² = 0.9992"), table_cell_style)],

        [Paragraph(th("<b>Computer Vision</b>"), table_cell_bold),
         Paragraph(th("Dual-band Red HSV + Morphological 5x5 + Hybrid Edge"), table_cell_style),
         Paragraph(th("สกัดสีเป้าหมายอย่างแม่นยำแม้ในสภาพแสงรบกวน พร้อมเส้นขอบคมชัด"), table_cell_style)],

        [Paragraph(th("<b>Shape Classification</b>"), table_cell_bold),
         Paragraph(th("approxPolyDP + Aspect Ratio + Extent + Fill Ratio"), table_cell_style),
         Paragraph(th("แยกแยะรูปทรง 4 แบบ (Circle, Square, Rect_H, Rect_V) ถูกต้อง 100%"), table_cell_style)],

        [Paragraph(th("<b>Gimbal Servoing</b>"), table_cell_bold),
         Paragraph(th("3-Tier Adaptive Speed Closed-Loop PID"), table_cell_style),
         Paragraph(th("เล็งเป้ารวดเร็วเมื่อระยะไกล (540°/s) และนุ่มนวลละเอียดเมื่อใกล้จุดเล็ง ±18 px"), table_cell_style)],

        [Paragraph(th("<b>Anti-Double Fire</b>"), table_cell_bold),
         Paragraph(th("Dynamic ROI Masking (+15 px Margin Blackout)"), table_cell_style),
         Paragraph(th("ป้องกันการเล็งซ้ำเป้าหมายที่ยิงล้มแล้วตลอดช่วงการทำงานของกำแพงนั้น"), table_cell_style)],

        [Paragraph(th("<b>SLAM Navigation</b>"), table_cell_bold),
         Paragraph(th("Grid SLAM + DFS Frontier Priority + BFS Backtrack"), table_cell_style),
         Paragraph(th("สำรวจเขาวงกตครบทุกช่อง และเดินทางกลับจุด (0, 0) ด้วยเส้นทางที่สั้นที่สุด"), table_cell_style)]
    ]
    tech_table = Table(tech_data, colWidths=[42*mm, 60*mm, 72*mm])
    tech_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), PRIMARY),
        ('GRID', (0, 0), (-1, -1), 0.5, BORDER_COLOR),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, CARD_BG]),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('PADDING', (0, 0), (-1, -1), 3),
    ]))
    story.append(tech_table)

    # PAGE 5: SYSTEM CONSTANTS, OUTPUTS & CONCLUSION
    story.append(PageBreak())
    story.append(Paragraph(th("<b>ตารางค่าคงที่และพารามิเตอร์สำคัญของโค้ดระบบ (System Configuration Constants)</b>"), h2_style))
    const_data = [
        [Paragraph(th("<b>พารามิเตอร์</b>"), table_header_style),
         Paragraph(th("<b>ค่าที่ตั้งไว้</b>"), table_header_style),
         Paragraph(th("<b>คำอธิบายการทำงาน</b>"), table_header_style),
         Paragraph(th("<b>พารามิเตอร์</b>"), table_header_style),
         Paragraph(th("<b>ค่าที่ตั้งไว้</b>"), table_header_style),
         Paragraph(th("<b>คำอธิบายการทำงาน</b>"), table_header_style)],

        [Paragraph(th("<code>CELL_SIZE_CM</code>"), table_cell_bold),
         Paragraph(th("60.0 cm"), table_cell_style),
         Paragraph(th("ขนาดของช่องตารางกริด"), table_cell_style),
         Paragraph(th("<code>WALL_THRESHOLD</code>"), table_cell_bold),
         Paragraph(th("52.0 cm"), table_cell_style),
         Paragraph(th("เกณฑ์ระยะ ToF ตัดสินกำแพง"), table_cell_style)],

        [Paragraph(th("<code>AIM_ACCEPT_PX</code>"), table_cell_bold),
         Paragraph(th("18.0 px"), table_cell_style),
         Paragraph(th("ความเผื่อการเล็งกึ่งกลาง"), table_cell_style),
         Paragraph(th("<code>BURST_FIRE_COUNT</code>"), table_cell_bold),
         Paragraph(th("2 นัด"), table_cell_style),
         Paragraph(th("จำนวนกระสุนยิงต่อเป้าหมาย"), table_cell_style)],

        [Paragraph(th("<code>KP_YAW_HOLD</code>"), table_cell_bold),
         Paragraph(th("0.8 / 1.2"), table_cell_style),
         Paragraph(th("Gain คุมทิศทางการเดินตรง"), table_cell_style),
         Paragraph(th("<code>DECEL_ZONE_M</code>"), table_cell_bold),
         Paragraph(th("0.18 m"), table_cell_style),
         Paragraph(th("ระยะเริ่มชะลอก่อนถึงจุดกึ่งกลาง"), table_cell_style)],

        [Paragraph(th("<code>SHARP_PUSH_DIST</code>"), table_cell_bold),
         Paragraph(th("11.0 cm"), table_cell_style),
         Paragraph(th("ระยะ Guardrail ผลักออก"), table_cell_style),
         Paragraph(th("<code>MAX_ROUND1_TIME</code>"), table_cell_bold),
         Paragraph(th("570.0 s"), table_cell_style),
         Paragraph(th("เวลาจำกัดรอบที่ 1 (9.5 นาที)"), table_cell_style)]
    ]
    const_table = Table(const_data, colWidths=[31*mm, 18*mm, 38*mm, 31*mm, 18*mm, 38*mm])
    const_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), SECONDARY),
        ('GRID', (0, 0), (-1, -1), 0.5, BORDER_COLOR),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, CARD_BG]),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('PADDING', (0, 0), (-1, -1), 3.5),
    ]))
    story.append(const_table)
    story.append(Spacer(1, 4*mm))

    # SECTION 8: MISSION OUTPUTS & ROUND 2 HANDOFF
    story.append(Paragraph(th("8. ข้อมูลผลลัพธ์และการส่งออกไฟล์ข้อมูล (Generated Outputs & Artifacts)"), h1_style))
    story.append(HRFlowable(width="100%", thickness=1, color=PRIMARY, spaceAfter=5))

    outputs_desc = (
        "เมื่อสิ้นสุดการทำงานของสคริปต์ <code>run_round1.py</code> โค้ดจะส่งออกไฟล์ข้อมูลอัตโนมัติเพื่อใช้บันทึกและประมวลผลต่อ:\n"
        "1. <b><code>Map_Data_Round1.json</code></b>: ไฟล์ JSON บันทึกโครงสร้างแผนที่เขาวงกต พิกัดกำแพงทุกช่องสถานะ และตำแหน่งของเป้าหมายทั้งหมดที่ตรวจพบและยิงแล้ว\n"
        "2. <b><code>exploration_telemetry.csv</code></b>: บันทึกข้อมูล Telemetry ประวัติการก้าวเดิน เวลา พิกัด (gx, gy), มุม Heading, ค่าระยะ ToF 4 ทิศ และสถานะกำแพงทุก Step\n"
        "3. <b><code>exploration_log.txt</code></b>: บันทึกประวัติเหตุการณ์ (Event Logs), ลำดับการเปลี่ยนสถานะ State Machine และข้อผิดพลาดทั้งหมด\n"
        "4. <b><code>detected_targets/</code></b>: โฟลเดอร์เก็บภาพถ่ายหลักฐานเป้าหมายที่เล็งยิงพร้อมกรอบ Bounding Box และ Class Label"
    )
    story.append(Paragraph(th(outputs_desc), body_style))
    story.append(Spacer(1, 4*mm))

    summary_box = (
        "<b>สรุปการทำงานของชุดโค้ดโปรแกรม (Technical Summary):</b><br/>"
        "ชุดโค้ดโปรแกรมสำหรับหุ่นยนต์ RoboMaster EP ได้รับการออกแบบเชิงโมดูลาร์ (Modular Architecture) "
        "โดยแบ่งแยกหน้าที่ระหว่างโมดูล Calibration, Computer Vision, SLAM Path Planning และ Motion Control ไว้อย่างเป็นระบบ "
        "ทำให้โค้ดมีความยืดหยุ่น สามารถบำรุงรักษาและปรับแต่งพารามิเตอร์ได้อย่างรวดเร็ว "
        "พร้อมรองรับการทำงานอัตโนมัติอย่างมีเสถียรภาพและแม่นยำสูง"
    )
    story.append(make_callout(summary_box, bg_color=SUCCESS_BG, border_color=SUCCESS_BORDER))

    # Build Document
    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"PDF Successfully Generated: {pdf_path}")
    return pdf_path

if __name__ == "__main__":
    build_pdf()

