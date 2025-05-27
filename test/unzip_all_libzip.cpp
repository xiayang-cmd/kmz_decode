#include <zip.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstring>
#include <climits>          // 为了 PATH_MAX

// 递归创建目录（极简实现，够测试用）
static void mkdir_p(const char* path) {
    char tmp[PATH_MAX];
    std::strcpy(tmp, path);
    for (char* p = tmp + 1; *p; ++p) {
        if (*p == '/') { *p = '\0'; mkdir(tmp, 0755); *p = '/'; }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::puts("用法: unzip_test <zip文件> <解压到的目录>");
        return 0;
    }

    // 1. 打开 zip
    int err = 0;
    zip_t* za = zip_open(argv[1], ZIP_RDONLY, &err);
    if (!za) { std::fprintf(stderr, "zip_open 失败, err=%d\n", err); return 1; }

    // 2. 遍历条目
    zip_int64_t total = zip_get_num_entries(za, 0);
    for (zip_uint64_t i = 0; i < (zip_uint64_t)total; ++i) {
        const char* name = zip_get_name(za, i, 0);
        if (!name) continue;

        // 拼出目标路径
        char dst[PATH_MAX];
        std::snprintf(dst, sizeof(dst), "%s/%s", argv[2], name);

        // 目录：名字以 '/' 结尾
        if (name[std::strlen(name) - 1] == '/') {
            mkdir_p(dst);
            continue;
        }

        // 3. 创建父目录 + 写文件
        mkdir_p(dst);
        zip_file_t* zf = zip_fopen_index(za, i, 0);
        if (!zf) { std::fprintf(stderr, "无法读取 %s\n", name); continue; }

        FILE* fp = std::fopen(dst, "wb");
        if (!fp) { zip_fclose(zf); std::perror(dst); continue; }

        char buf[8192];
        zip_int64_t n;
        while ((n = zip_fread(zf, buf, sizeof(buf))) > 0)
            std::fwrite(buf, 1, n, fp);

        std::fclose(fp);
        zip_fclose(zf);
    }
    zip_close(za);
    std::puts("解压完成！");
}
