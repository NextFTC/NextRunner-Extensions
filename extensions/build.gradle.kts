plugins {
    id("com.android.library") version "8.0.2" apply false

    kotlin("android") version "2.0.0" apply false

    id("org.jetbrains.dokka") version "2.0.0"

    id("io.deepmedia.tools.deployer") version "0.18.0"
}

buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath("com.android.tools.build:gradle:8.0.2")
        classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.9.0")
        // classpath("androidx.navigation:navigation-safe-args-gradle-plugin:2.7.0") // Example
    }
}