%{
#include "Wrl_lexer.hpp"


#if (defined _MSC_VER)
#pragma warning ( disable : 4786 )
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>


#define YYERROR_VERBOSE 1
#define YYDEBUG         1

extern int yylex();
extern void yyerror(const char * s);

 std::string * coord_str_ptr = 0;
 std::string * coord_index_str_ptr = 0;
 
%}

%union {
  std::string * text;
}

%type <text> NUMBER
%type <text> IDENTIFIER
%type <text> Id nodeTypeId fieldId
%type <text> sfValues sfValue
%type <text> sfint32Values

%token IDENTIFIER
%token NUMBER

%token LS
%token RS
%token AT
%token AND
%token OR
%token LE
%token GE
%token EQ
%token NE

%start Start

%%

/* General: */

Start           : vrmlScene
                ;

vrmlScene       : statements
                ; 

statements      : /* empty */
                | statements statement
                ;

statement       : nodeStatement
                ;

nodeStatement   : node 
                ;

/* nodes: */

node            : nodeTypeId '{' nodeBody '}'
                ;

nodeBody        : /* empty */
                | nodeBody fieldId sfValue
                | nodeBody fieldId '[' sfValues ']'
                {
                  if ((*$2) == "point") {
                    coord_str_ptr = $4;
                  } else if ((*$2) == "coordIndex") {
                    coord_index_str_ptr = $4;
                  }
                }
                | nodeBody fieldId sfnodeValue
                ; 


nodeTypeId      : Id 
                ; 

fieldId         : Id 
                ; 

Id              : IDENTIFIER
                ;

/* Fields: */

sfValue         : sfint32Values { $$ = $1; }
                ;

sfnodeValue     : nodeStatement
                ; 

sfint32Values   : NUMBER { $$ = $1; }
                | sfint32Values NUMBER { (*$1) += " " + (*$2); $$ = $1; }
                ; 

sfValues        : sfint32Values { $$ = $1; }
                ;

%%

/*! global yyerror */
void yyerror(const char * message)
{
  Wrl_lexer::instance()->yyerror(message, (int) yychar);
}

/*! global yylex */
int yylex(void)
{
  return Wrl_lexer::instance()->yylex();
}
