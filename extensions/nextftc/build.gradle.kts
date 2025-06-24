plugins {
    id("com.android.library")

    kotlin("android")

    id("org.jetbrains.dokka") version "2.0.0"
    id("io.deepmedia.tools.deployer") version "0.18.0"
}

android {
    namespace = "com.acmerobotics.roadrunner.extensions"
    //noinspection GradleDependency
    compileSdk = 33

    defaultConfig {
        minSdk = 24

        testInstrumentationRunner = "android.support.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += ("-Xjvm-default=all")
    }

    testOptions {
        unitTests {
            isReturnDefaultValues = true
        }
    }
}


repositories {
    mavenCentral()
    google()
    maven("https://maven.rowanmcalpin.com/")
    maven("https://maven.brott.dev/")
}

dependencies {
    implementation(libs.bundles.nextrunner)
    api(libs.bundles.nextftc)
    api(libs.bundles.ftcsdk)

    implementation(libs.bundles.dashboard)
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier.set("html-docs")
}

deployer {
    projectInfo {
        groupId.set("dev.nextftc.nextrunner.extensions")
        artifactId.set("nextftc")

        name.set("NextRunner-Extensions")
        description.set("Compatibility libraries for NextRunner.")
        url.set("https://github.com/NextFTC/NextRunner")
        scm {
            fromGithub("NextFTC", "NextRunner-Extensions")
        }
        license("GNU General Public License, version 3", "https://www.gnu.org/licenses/gpl-3.0.html")

        developer("Zachary Harel", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
        developer("Ryan Brott", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
    }

    signing {
        key.set(secret("MVN_GPG_KEY"))
        password.set(secret("MVN_GPG_PASSWORD"))
    }

    content {
        kotlinComponents {
            kotlinSources()
            docs(dokkaJar)
        }
    }

    localSpec {
        release.version.set("$version")
    }

    nexusSpec("snapshot") {
        release.version.set("$version")
        repositoryUrl.set("https://central.sonatype.com/repository/maven-snapshots/")
        auth {
            user.set(secret("SONATYPE_USERNAME"))
            password.set(secret("SONATYPE_PASSWORD"))
        }
    }

    centralPortalSpec {
        auth {
            user.set(secret("SONATYPE_USERNAME"))
            password.set(secret("SONATYPE_PASSWORD"))
        }
        allowMavenCentralSync.set((property("automaticMavenCentralSync") as String).toBoolean())
    }
}
