import os
import codecs
import chardet
import argparse

def detect_encoding(file_path):
    """检测文件编码"""
    with open(file_path, 'rb') as f:
        raw_data = f.read()
        result = chardet.detect(raw_data)
        return result['encoding']

def convert_file_to_utf8(file_path, backup=False, verbose=False):
    """将文件转换为UTF-8编码"""
    try:
        # 检测当前编码
        encoding = detect_encoding(file_path)
        
        if encoding is None:
            if verbose:
                print(f"无法检测 {file_path} 的编码，跳过")
            return False
        
        if encoding.lower() == 'utf-8' or encoding.lower() == 'utf-8-sig':
            if verbose:
                print(f"{file_path} 已经是UTF-8编码，跳过")
            return True
        
        # 创建备份
        if backup:
            backup_path = file_path + '.bak'
            with open(file_path, 'rb') as src, open(backup_path, 'wb') as dst:
                dst.write(src.read())
        
        # 读取文件内容
        with codecs.open(file_path, 'r', encoding=encoding, errors='ignore') as f:
            content = f.read()
        
        # 以UTF-8写入
        with codecs.open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        if verbose:
            print(f"已将 {file_path} 从 {encoding} 转换为 UTF-8")
        return True
        
    except Exception as e:
        print(f"转换 {file_path} 时出错: {e}")
        return False

def process_directory(directory, backup=False, verbose=False, dry_run=False):
    """递归处理目录中的所有.cs文件"""
    converted_count = 0
    error_count = 0
    
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.lower().endswith('.cs'):
                file_path = os.path.join(root, file)
                
                if verbose and not dry_run:
                    print(f"处理文件: {file_path}")
                
                if not dry_run:
                    success = convert_file_to_utf8(file_path, backup, verbose)
                    if success:
                        converted_count += 1
                    else:
                        error_count += 1
                else:
                    # 在dry_run模式下只显示将要处理的文件
                    encoding = detect_encoding(file_path)
                    if encoding and encoding.lower() not in ['utf-8', 'utf-8-sig']:
                        print(f"将要转换: {file_path} (当前编码: {encoding})")
                        converted_count += 1
    
    return converted_count, error_count

def main():
    parser = argparse.ArgumentParser(description='递归将文件夹下的.cs文件编码格式改为UTF-8')
    parser.add_argument('-d', '--directory', default='.', 
                       help='要处理的目录，默认为当前目录')
    parser.add_argument('-b', '--backup', action='store_true',
                       help='转换前创建备份文件（.bak扩展名）')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='显示详细输出')
    parser.add_argument('-n', '--dry-run', action='store_true',
                       help='试运行，不实际修改文件，只显示将要进行的操作')
    
    args = parser.parse_args()
    
    print(f"开始处理目录: {args.directory}")
    print(f"备份文件: {'是' if args.backup else '否'}")
    print(f"试运行: {'是' if args.dry_run else '否'}")
    print("-" * 50)
    
    if args.dry_run:
        print("试运行模式 - 不会实际修改文件")
    
    converted_count, error_count = process_directory(
        args.directory, args.backup, args.verbose, args.dry_run
    )
    
    print("-" * 50)
    if args.dry_run:
        print(f"试运行完成：{converted_count} 个文件需要转换")
    else:
        print(f"处理完成：成功转换 {converted_count} 个文件，{error_count} 个文件失败")

if __name__ == "__main__":
    main()