import os
import re

def add_file_header(file_path, package_name, package_root):
    """
    给指定文件添加头部注释，标明文件位置
    :param file_path: 文件路径
    :param package_name: 包名
    :param package_root: package根目录路径
    """
    # 构造注释内容，相对于package根目录的路径
    relative_path = os.path.relpath(file_path, package_root)
    header_comment = f"{package_name}/{relative_path}"
    
    # 读取文件内容
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 检查是否已经存在头部注释
    if content.startswith('# ' + header_comment) or content.startswith('// ' + header_comment):
        print(f"File {file_path} already has header comment, skipping...")
        return
    
    # 根据文件类型确定注释符号
    if file_path.endswith('.py'):
        comment_prefix = '# '
    elif file_path.endswith('.cpp'):
        comment_prefix = '// '
    else:
        return
    
    # 添加头部注释
    new_content = comment_prefix + header_comment + '\n' + content
    
    # 写入文件
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    
    print(f"Added header comment to {file_path}")

def find_package_root(file_path):
    """
    查找文件所属的package根目录（包含package.xml的目录）
    :param file_path: 文件路径
    :return: package根目录路径，如果未找到则返回None
    """
    current_dir = os.path.dirname(file_path)
    while current_dir != '/':
        package_xml_path = os.path.join(current_dir, 'package.xml')
        if os.path.exists(package_xml_path):
            return current_dir
        current_dir = os.path.dirname(current_dir)
    return None

def get_package_name(package_root):
    """
    获取package名称（目录名）
    :param package_root: package根目录路径
    :return: package名称
    """
    return os.path.basename(package_root)

def process_directory(src_dir):
    """
    递归处理目录下的所有.py和.cpp文件
    :param src_dir: 源代码目录路径
    """
    # 遍历目录中的所有文件
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            if file.endswith('.py') or file.endswith('.cpp'):
                file_path = os.path.join(root, file)
                
                # 查找该文件所属的package根目录
                package_root = find_package_root(file_path)
                if package_root is None:
                    print(f"Warning: No package.xml found for {file_path}, skipping...")
                    continue
                
                # 获取package名称
                package_name = get_package_name(package_root)
                # 修改: 将package_root传递给add_file_header函数
                add_file_header(file_path, package_name, package_root)

if __name__ == "__main__":
    # 假设脚本运行在包含src目录的位置
    current_dir = os.getcwd()
    src_directory = os.path.join(current_dir, "src")
    
    if os.path.exists(src_directory):
        process_directory(src_directory)
    else:
        print("Error: src directory not found!")