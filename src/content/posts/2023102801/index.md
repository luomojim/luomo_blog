---
title: 文章样式
published: 2023-10-28
pinned: false
description: 一个关于写markdown文章的格式范例.
tags: [markdown, bloging]
category: 样例
licenseName: "Unlicensed"
author: emn178
sourceLink: "https://github.com/emn178/markdown"
draft: false
date: 2023-10-28
pubDate: 2023-10-28
permalink: "encrypted-example"
---

# 创建文章
在src/content/posts目录下创建一个新的文件夹，文件夹名应该具有描述性，例如my-complex-post。

在新创建的文件夹中创建一个名为index.md的文件。

在index.md文件中添加frontmatter（前置元数据），这是文章的配置信息，必须包含title和description字段：


# Frontmatter字段详解
frontmatter支持的字段包括：

## 必需字段
- title：文章标题（必需）
- description：文章描述（必需）
## 发布相关
- published：文章发布日期，格式为YYYY-MM-DD
- pubDate：文章发布日期（与published类似）
- date：文章创建日期
- draft：是否为草稿，true表示草稿，false表示正式发布
- permalink: 固定链接
## 内容分类
- tags：文章标签数组，用于标记文章主题
- category：文章分类，用于组织文章
- pinned：是否置顶文章，true表示置顶
## 作者信息
- author：文章作者姓名
- licenseName：文章许可证名称，如"MIT"、"CC BY 4.0"等
- sourceLink：文章源链接，通常指向GitHub仓库或原始来源
## 图片设置
- image：文章封面图片
在frontmatter下方编写文章内容，可以使用标准的Markdown语法。