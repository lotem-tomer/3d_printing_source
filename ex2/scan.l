%option yyclass="Wrl_lexer"
%option never-interactive

%{

#if (defined _MSC_VER)
#pragma warning ( disable : 4786 )
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "Wrl_lexer.hpp"

// For older versions of flex (less than 2.5.31)
// extern "C" int yywrap(void);

using std::cin;
using std::cerr;
using std::cout;

#include "parse.hpp"

Wrl_lexer * Wrl_lexer::s_instance = NULL;

#ifdef _DEBUG
#define RET(x) \
{ \
  if (yy_flex_debug) printf("lex %hu [%s]\n", (int) x, YYText()); \
  return(x); \
}
#else
#define RET(x) return(x)
#endif

%}

I			[1-9]
D			[0-9]
O			[0-7]
L			[a-zA-Z]
H			[a-fA-F0-9]
E			[Ee][+-]?{D}+
FS			(f|F|l|L)
IS			(u|U|l|L)*

integer			(0|({I}{D}*))
mantissa		(({integer}\.{D}*)|(\.{D}+)|({integer}))

number 			{mantissa}{E}?
hexnumber 		(0[xX]{H}+)
octnumber 		(0{O}+)

idStartChar		([a-zA-Z\$_\x80-\xFF])
idRestChar      	({idStartChar}|[0-9]|\-)

%s vrml js

%%

<INITIAL>"#VRML V2.0"[^\n]*	{ BEGIN(vrml); }

<vrml>"/*"		{ comment(); }
<vrml>"#"		{ comment_to_eol(); }

<vrml>([+/-]?{number}|{hexnumber}|{octnumber})		{ yylval.text = new std::string(YYText()); RET(NUMBER); }


<vrml>{idStartChar}{idRestChar}*	{ yylval.text = new std::string(YYText()); RET(IDENTIFIER); }

";"			{ RET(';'); }
<vrml>"("		{ RET('('); }
<vrml>")"		{ RET(')'); }
":"			{ RET(':'); }
"|"			{ RET('|'); }
<vrml>"{"		{ RET('{'); }
<vrml>"}"		{ RET('}'); }
<vrml>"["		{ RET('['); }
<vrml>"]"		{ RET(']'); }
"?"			{ RET('?'); }
"="			{ RET('='); }
"*"			{ RET('*'); }
"/"			{ RET('/'); }
"+"			{ RET('+'); }
"-"			{ RET('-'); }
"~"			{ RET('~'); }
"@"			{ RET('@'); }
"&"			{ RET('&'); }
"^"			{ RET('^'); }
"%"			{ RET('%'); }
"."			{ RET('.'); }

"<<"			{ RET(LS); }
">>"			{ RET(RS); }
"@@"			{ RET(AT); }
"&&"            	{ RET(AND); }
"||"            	{ RET(OR); }
"<="            	{ RET(LE); }
">="            	{ RET(GE); }
"=="            	{ RET(EQ); }
"!="            	{ RET(NE); }

\n			{ m_lineno++; }		/* new line */ 
[ \t\v\f,]		{ ; }		    	/* skip blanks and tabs */
.			{ RET(yytext[0]); }  	/* everything else */
%%

// extern "C" int yywrap()
int yyFlexLexer::yywrap() { return(1); }

void Wrl_lexer::comment()
{
  register int c;

  for ( ; ; ) {
    /* eat up text of comment: */
    while ((c = yyinput()) != '*' && c != EOF) {
      if (c == '\n') m_lineno++;
    }

    if (c == '*') {
      while ((c = yyinput()) == '*');
      if (c == '/') break;    /* found the end */
    }

    if (c == EOF) {
      fprintf(stderr, "EOF in comment!\n");
      break;
    }
  }
}

// For older versions of flex (less than 2.5.31)
#ifndef yytext_ptr
#define yytext_ptr yytext
#endif

void Wrl_lexer::comment_to_eol()
{
  char c;
  while ((c = yyinput()) != '\n' && c != 0) /* putchar(c) */ ;
  if (c == '\n') unput(c);
  else /* putchar(c) */ ;
}

void Wrl_lexer::yyerror(const char * message, int cur_token)
{
  fprintf(stderr, "Syntax error on line %d token %c\n", m_lineno, cur_token);
  exit(-1);
}
