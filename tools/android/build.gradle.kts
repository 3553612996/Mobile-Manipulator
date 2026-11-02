// 顶层构建脚本：只声明插件版本，配置在各模块里。
// AGP 9 自带 Kotlin 支持（内置 KGP 2.2.10），不能再叠加 org.jetbrains.kotlin.android。
plugins {
    id("com.android.application") version "9.2.1" apply false
    id("org.jetbrains.kotlin.plugin.compose") version "2.2.10" apply false
}
