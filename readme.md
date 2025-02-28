## このリポジトリについて
RMC-RA4M1(マイコンカー用のRA4M1ボード)向けのプログラムです。  
vscode + platform ioで開発します。

### ドキュメント
基板の仕様、開発環境、ソフトの解説等は以下のドキュメントサイトで公開しています。  
https://7-rate.github.io/mcr-pro-docs/


### コーディング規約
1. あらゆる名前はスネークスケールで書くこと。  
   スネークスケールとは、単語の間をアンダースコアで区切る書き方のことです。  
   単語は全て小文字で書きます。  
   OK 例：`int sample_variable = 0;`  
   NG 例：`int sampleVariable = 0;`  
   ただし、定数は全て大文字で書きます。

2. 関数単位でコメントを書くこと。  
   以下のフォーマットでコメントを書いてください。  
   備考については省略可能です。  
   引数や戻り値がある場合は、それらの取りうる値についても明確になるように記載してください。

```
/*
 * 概要：
 * 引数：
 * 戻り値：
 * 詳細：
 * 備考：
 */
```

3. ファイルを追加する場合は以下のテンプレートを使うこと。
<details><summary>cppファイルの場合</summary><div>

```cpp
/*
 * 概要：
 */
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/

/***********************************/
/* Local Variables                 */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/

/***********************************/
/* Class implementions             */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/
```

</div></details>

<details><summary>hファイルの場合</summary><div>

```h
/*
 * 概要：
 */
/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Global definitions              */
/***********************************/

/***********************************/
/* Class                           */
/***********************************/

/***********************************/
/* Global functions                */
/***********************************/

/***********************************/
/* Global Variables                */
/***********************************/
```

</div></details>

### 貢献方法
issue/PRを歓迎します。

## ライセンス
このリポジトリはMITライセンスです。  
詳しくは[LICENSE](LICENSE)を参照してください。  
