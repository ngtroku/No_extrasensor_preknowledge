import shutil
from pathlib import Path

def cleanup_intermediate_pcds(pcd_dir_path):

    target_dir = Path(pcd_dir_path)
    
    base_dir = target_dir.parent

    if base_dir.exists() and base_dir.is_dir():
        print(f"\n[Postprocess] 🧹 Cleaning up all intermediate files...")
        try:
            shutil.rmtree(base_dir)
            print(f"🗑️  [Success] Removed entire directory and its contents: {base_dir.resolve()}")
        except Exception as e:
            print(f"❌  [Error] Failed to remove '{base_dir}': {e}")
    else:
        print(f"\n[Postprocess] No intermediate directory found at '{base_dir}'.")

if __name__ == "__main__":
    cleanup_intermediate_pcds(pcd_dir_path="pcd_output/spoofed")