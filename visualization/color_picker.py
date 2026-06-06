#!/usr/bin/env python3
"""
color_picker.py
===============
Color configuration guide and reference script for the Master Thesis plots.
Maintains semantic consistency and visual design guidelines.

Guidelines & Principles
-----------------------
1. Statistical Metric Isolation:
   - MAE is strictly mapped to Paul Tol High-Contrast Blue (#004488).
   - RMSE is strictly mapped to Paul Tol High-Contrast Red (#BB5566).
   - Avoid using Red or Blue for model features or sensor configurations to prevent visual confusion.

2. Feature Modalities:
   - EMG Only is mapped to Paul Tol Vibrant Orange (#EE7733).
   - IMU Only is mapped to Okabe-Ito Bluish Green (#009E73).
   - EMG+IMU (Fusion/All) is mapped to Paul Tol Bright Purple/Magenta (#AA3377).

3. Statistical Significance (Tukey HSD Pairwise comparisons):
   - *** p < 0.001: Vibrant Red (#CC3311)
   - **  p < 0.01:  Muted Purple (#AA4499)
   - *   p < 0.05:  Bright Blue (#4477AA)
   - n.s. p >= 0.05: Light Grey (#BBBBBB)
"""

# ──────────────────────────────────────────────────────────
# COLOR CONSTANTS
# ──────────────────────────────────────────────────────────

# ─── 1. Paul Tol Scientific Semantic Colors ───
COLOR_EMG = "#95356F"       # Dark Purple (EMG)
COLOR_IMU = "#007355"       # Forest Green (IMU)
COLOR_FUSION = "#006781"    # Teal (EMG+IMU)

# ─── 2. Consistent Statistical Metric Colors ───
COLOR_MAE = "#004488"       # Paul Tol High-Contrast Blue (MAE)
COLOR_RMSE = "#BB5566"      # Paul Tol High-Contrast Red (RMSE)
COLOR_R2 = "#774499"        # Paul Tol Muted Purple (R^2 Coefficient of Determination)
COLOR_GAP = "#C85200"       # Paul Tol Dark Orange (Generalization Gap)
COLOR_COUNT = "#4A5568"     # Muted Charcoal (sample count bars/lines)

# ─── 3. Reference and Boundary Colors ───
COLOR_UNITY = "#888888"     # Paul Tol Slate Grey (dashed perfect-prediction line)
COLOR_MEDIAN = "#DDAA33"    # Paul Tol High-Contrast Yellow (pop-out median line in boxplots)
COLOR_TARGET = "#228833"    # Paul Tol Bright Green (balanced dataset target threshold)
COLOR_OUTLIER = "#BBBBBB"   # Paul Tol Light Grey (scatter points)

# ─── 4. Pairwise Tukey HSD Significance Colors ───
COLOR_P_001 = "#CC3311"     # Paul Tol Vibrant Red (*** p < 0.001)
COLOR_P_01 = "#AA4499"      # Paul Tol Muted Purple (** p < 0.01)
COLOR_P_05 = "#4477AA"      # Paul Tol Bright Blue (* p < 0.05)
COLOR_P_NS = "#BBBBBB"      # Paul Tol Light Grey (n.s. p >= 0.05)


# ──────────────────────────────────────────────────────────
# COLOR PALETTE SPECIFICATIONS
# ──────────────────────────────────────────────────────────

COLORS = {
    # 1. Feature Modalities
    "EMG Only": {
        "hex": COLOR_EMG,
        "role": "EMG features and EMG-only configuration runs",
        "palette": "Dark Purple"
    },
    "IMU Only": {
        "hex": COLOR_IMU,
        "role": "IMU features and IMU-only configuration runs",
        "palette": "Forest Green"
    },
    "EMG + IMU": {
        "hex": COLOR_FUSION,
        "role": "Sensor fusion configuration runs / Full model",
        "palette": "Vibrant Orange"
    },

    # 2. Statistical Metric Colors
    "MAE": {
        "hex": COLOR_MAE,
        "role": "Mean Absolute Error lines, bars, and markers",
        "palette": "Paul Tol High-Contrast Blue"
    },
    "RMSE": {
        "hex": COLOR_RMSE,
        "role": "Root Mean Squared Error lines, bars, and markers",
        "palette": "Paul Tol High-Contrast Red"
    },
    "R2": {
        "hex": COLOR_R2,
        "role": "Coefficient of Determination (R^2) lines, bars, markers, or secondary axes",
        "palette": "Paul Tol Muted Purple"
    },
    "Generalization Gap": {
        "hex": COLOR_GAP,
        "role": "Generalization Gap (MAE) bars in modality comparison charts",
        "palette": "Paul Tol Dark Orange"
    },

    # 3. Pairwise Tukey HSD Significance Colors
    "p < 0.001 (***)": {
        "hex": COLOR_P_001,
        "role": "Extremely significant differences in error distributions",
        "palette": "Paul Tol Vibrant Red"
    },
    "p < 0.01 (**)": {
        "hex": COLOR_P_01,
        "role": "Highly significant differences in error distributions",
        "palette": "Paul Tol Muted Purple"
    },
    "p < 0.05 (*)": {
        "hex": COLOR_P_05,
        "role": "Statistically significant differences in error distributions",
        "palette": "Paul Tol Bright Blue"
    },
    "p >= 0.05 (n.s.)": {
        "hex": COLOR_P_NS,
        "role": "Non-significant differences in error distributions",
        "palette": "Paul Tol Light Grey"
    },

    # 4. Reference & Boundary Elements
    "Segment Count": {
        "hex": COLOR_COUNT,
        "role": "Muted background representation of sample counts",
        "palette": "Muted Charcoal"
    },
    "Perfect Prediction": {
        "hex": COLOR_UNITY,
        "role": "Ideal prediction boundary line (Unity Line y=x)",
        "palette": "Paul Tol Slate Grey"
    },
    "Median (Boxplots)": {
        "hex": COLOR_MEDIAN,
        "role": "High-contrast median line to draw attention inside boxplots",
        "palette": "Paul Tol High-Contrast Yellow"
    },
    "Balanced Target": {
        "hex": COLOR_TARGET,
        "role": "Original target sample threshold line in distribution checks",
        "palette": "Paul Tol Bright Green"
    },
    "Outliers": {
        "hex": COLOR_OUTLIER,
        "role": "Downsampled outlier points in regression scatter plots",
        "palette": "Paul Tol Light Grey"
    }
}


def hex_to_rgb(hex_str):
    """Converts a hex color string to an (R, G, B) tuple."""
    h = hex_str.lstrip('#')
    return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))


def show_palette():
    """Prints the color system directly in the terminal with colored blocks."""
    print("=" * 90)
    print("               MASTER THESIS PLOT COLOR PALETTE & SPECIFICATION GUIDE")
    print("=" * 90)
    print(f"{'Color Block':<14}  {'Semantic Name':<22}  {'Hex Code':<10}  {'Role & Usage':<40}")
    print("-" * 90)
    
    for name, details in COLORS.items():
        h = details["hex"]
        r, g, b = hex_to_rgb(h)
        # Output color block using ANSI TrueColor escape sequences
        block = f"\033[48;2;{r};{g};{b}m      \033[0m"
        print(f" {block:<13}  {name:<22}  {h:<10}  {details['role']}")
        
    print("=" * 90)
    print("DESIGN CONSIDERATIONS FOR ACADEMIC PLOTS:")
    print("1. Red-Green Avoidance: EMG (Orange) and IMU (Green/Teal) are chosen to remain readable")
    print("   for people with red-green colorblindness (deuteranopia/protanopia).")
    print("2. Contrast: Background boxplot fills should use soft tints (e.g. #FFF5EB for EMG,")
    print("   #F0F7F4 for IMU, and #FAF0F5 for Fusion) to ensure black text and labels remain readable.")
    print("3. Printability: Avoid relying on color alone; use line styles (solid vs dashed) and")
    print("   markers (circles vs squares) to identify metrics in black-and-white print.")
    print("=" * 90)



def save_palette_image(output_path="color_palette.png"):
    """Generates a high-quality visualization of the color palette using Matplotlib."""
    import matplotlib.pyplot as plt
    
    fig, ax = plt.subplots(figsize=(10, len(COLORS) * 0.45 + 1.0))
    ax.set_xlim(0, 10)
    ax.set_ylim(-0.5, len(COLORS) - 0.5)
    
    for idx, (name, details) in enumerate(reversed(list(COLORS.items()))):
        h = details["hex"]
        rect = plt.Rectangle((0.4, idx - 0.22), 1.5, 0.44, facecolor=h, edgecolor='#2C3E50', linewidth=0.8)
        ax.add_patch(rect)
        
        ax.text(2.1, idx, name, va='center', ha='left', fontweight='bold', color='#1A202C', fontsize=10)
        ax.text(4.6, idx, h, va='center', ha='left', fontfamily='monospace', color='#4A5568', fontsize=10)
        ax.text(5.8, idx, details['role'], va='center', ha='left', color='#4A5568', fontsize=9)
        
    plt.title("Master Thesis Plot Color Palette Specification", fontsize=12, fontweight='bold', color='#2C3E50', pad=15, loc='left')
    ax.set_axis_off()
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()


if __name__ == "__main__":
    from pathlib import Path
    show_palette()
    output_img = Path(__file__).resolve().parent / "color_palette.png"
    try:
        save_palette_image(output_img)
        print(f"\n[Success] Visual palette saved to: {output_img}")
    except Exception as e:
        print(f"\n[Warning] Could not generate visual palette image: {e}")
